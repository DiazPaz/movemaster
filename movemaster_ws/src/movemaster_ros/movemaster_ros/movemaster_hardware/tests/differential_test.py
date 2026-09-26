#!/usr/bin/env python3
"""Compare the compiled C++ codec to the untouched, user-validated Python module.

Usage: python3 tests/differential_test.py --oracle build/protocol_oracle
No CAN interface, python-can installation or hardware is needed.
"""
import argparse
import dataclasses
import importlib.util
import json
import math
from pathlib import Path
import random
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--oracle', type=Path, required=True)
parser.add_argument('--reference', type=Path, default=ROOT / 'tests/reference/sparkmax_json_protocol.py')
parser.add_argument('--spec', type=Path, default=ROOT / 'spec/spark-frames-2.1.0')
args = parser.parse_args()
module_spec = importlib.util.spec_from_file_location('spark_reference', args.reference)
ref = importlib.util.module_from_spec(module_spec)
sys.modules[module_spec.name] = ref
module_spec.loader.exec_module(ref)
protocol = ref.SparkMAXMotionProtocol(args.spec)
queries, expected = [], []
rng = random.Random(260926)

def clean(value):
    if dataclasses.is_dataclass(value):
        return clean(dataclasses.asdict(value))
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None  # JSON representation in the C++ oracle, not in the binary codec.
    return value

def add(query, call):
    queries.append(query)
    try:
        expected.append({'ok': True, 'value': clean(call())})
    except Exception:
        expected.append({'ok': False})

def packet(p):
    return clean(p)

# Every frame in the supplied file: constraints, endian/bit positions, signed/scaled fields.
for name, frame in protocol.frames.items():
    for sample in range(3):
        values = {}
        for key, spec in frame.signals.items():
            kind, bits = spec['type'].lower(), spec['lengthBits']
            scale, offset = spec.get('decodeScaleFactor', 1), spec.get('offset', 0)
            if kind == 'float':
                values[key] = (-13.75, 0.0, 321.0625)[sample]
            elif kind in ('bool', 'boolean'):
                values[key] = bool(sample % 2)
            else:
                low = spec.get('encodedMin', -(1 << (bits - 1)) if kind == 'int' else 0)
                high = spec.get('encodedMax', (1 << (bits - (kind == 'int'))) - 1)
                encoded = (low, rng.randint(low, high), high)[sample]
                values[key] = encoded * scale + offset
        add({'op': 'frame', 'frame': name, 'values': values, 'require_all': True},
            lambda f=frame, v=values: list(f.encode_payload(v, require_all=True)))
    raw = bytes(rng.getrandbits(8) for _ in range(frame.length_bytes))
    add({'op': 'decode', 'frame': name, 'data': list(raw)}, lambda f=frame, r=raw: f.decode_payload(r))

for name in ('PARAMETER_WRITE', 'MAXMOTION_POSITION_SETPOINT', 'STATUS_0'):
    add({'op': 'frame', 'frame': name, 'values': {}, 'require_all': True},
        lambda name=name: protocol.frames[name].encode_payload({}, require_all=True))
    add({'op': 'decode', 'frame': name, 'data': [0]}, lambda name=name: protocol.frames[name].decode_payload(b'\0'))

# Python ties-to-even, sign extension and the 64-bit boundary are easy migration regressions.
for bits in (1, 8, 16, 32, 64):
    for kind in ('int', 'uint'):
        spec = {'type': kind, 'lengthBits': bits}
        for value in (-129, -128, -2.5, -1.5, -0.5, 0, 0.5, 1.5, 2.5, 127, 255, 2**32-1, 2**63, 2**64-1):
            add({'op': 'encode_bits', 'spec': spec, 'value': value}, lambda s=spec, v=value: ref.SignalCodec.encode_bits(s, v))
        for raw in (0, 1, (1 << bits)-1, 1 << (bits-1)):
            add({'op': 'decode_bits', 'spec': spec, 'raw': raw}, lambda s=spec, v=raw: ref.SignalCodec.decode_bits(s, v))
for spec in ({'type': 'float', 'lengthBits': 16}, {'type': 'uint', 'lengthBits': 8, 'isBigEndian': True},
             {'type': 'uint', 'lengthBits': 8, 'decodeScaleFactor': 0}, {'type': 'unknown', 'lengthBits': 8}):
    add({'op': 'encode_bits', 'spec': spec, 'value': 1}, lambda s=spec: ref.SignalCodec.encode_bits(s, 1))

for device in (0, 1, 3, 6, 63):
    p = ref.SparkMAXMotionProtocol(args.spec, device_id=device)
    for slot in range(4):
        for value, ff, units in ((0.5, 0, 0), (-2.75, 0.5, 0), (13.125, -0.25, 1)):
            add({'op': 'setpoint', 'device_id': device, 'slot': slot, 'setpoint': value, 'ff': ff, 'units': units},
                lambda p=p, slot=slot, value=value, ff=ff, units=units: packet(p.maxmotion_setpoint_packet(
                    value, slot=slot, arbitrary_feedforward=ff, arbitrary_feedforward_units=units)))
    for group in ('pidf', 'maxmotion'):
        for slot in range(4):
            for key, definition in p[group][slot].items():
                value = 0.125 if group == 'pidf' else 125.5
                add({'op': 'parameter', 'device_id': device, 'group': group, 'slot': slot, 'key': key, 'value': value},
                    lambda p=p, d=definition, v=value: {'definition': d, 'write': packet(p.parameter_write_packet(d, v)),
                                                       'read': packet(p.parameter_read_packet(d))})
    add({'op': 'describe', 'device_id': device}, p.describe)
    for name, values in (('READ_PARAMETER_12_AND_13', {}), ('MAXMOTION_POSITION_SETPOINT', {'SETPOINT': 0.5})):
        add({'op': 'socketcan_roundtrip', 'device_id': device, 'frame': name, 'values': values},
            lambda p=p, n=name, v=values: packet(p.frames[n].packet(device, v)))

for alias in ('cruisevelocity', 'maxvelocity', ' Max Velocity ', 'max-accel', 'allowed_error', 'allowed_closed_loop_error'):
    d = protocol['maxmotion'][3][alias]
    add({'op': 'parameter', 'group': 'maxmotion', 'slot': 3, 'key': alias, 'value': 23},
        lambda d=d: {'definition': d, 'write': packet(protocol.parameter_write_packet(d, 23)),
                     'read': packet(protocol.parameter_read_packet(d))})
for kind in ('float', 'int', 'uint', 'bool', 'boolean'):
    for value in (-2**31, -1, 0, 1, 0.123, 2**32-1):
        add({'op': 'pack', 'value': value, 'type': kind}, lambda v=value, k=kind: protocol.pack_parameter_value(v, k))
    for raw in (0, 1, 0x80000000, 0xffffffff, 0x3f800000):
        add({'op': 'unpack', 'raw': raw, 'type': kind}, lambda r=raw, k=kind: protocol.unpack_parameter_value(r, k))
for group in ('pidf', 'maxmotion'):
    for slot in range(4):
        for key, d in protocol[group][slot].items():
            data = struct.pack('<ff', 1.25, -2.5)
            add({'op': 'read_response', 'group': group, 'slot': slot, 'key': key, 'data': list(data)},
                lambda d=d, data=data: protocol.decode_parameter_read_response(d, data))
for type_code in (1, 2, 3, 4, 0, 99):
    for result in (0, 1, 5):
        data = bytes([13, type_code]) + struct.pack('<I', 0x3dcccccd) + bytes([result])
        add({'op': 'write_response', 'data': list(data)}, lambda d=data: protocol.decode_parameter_write_response(d))

class Bus:
    def __init__(self, responses): self.responses = list(responses)
    def send(self, packet): pass
    def recv(self, timeout):
        if not self.responses: return None
        return type('Message', (), {'is_extended_id': True,
            'arbitration_id': protocol.frames['PARAMETER_WRITE_RESPONSE'].arbitration_id(1),
            'data': self.responses.pop(0)})()
# Adapt only python-can's transport conversion; codec and helper logic stay untouched.
ref.CANPacket.to_python_can = lambda self: self
for value in (0.1, 0.125):
    for verify in (True, False):
        for result in (0, 1):
            responses = [bytes([14, 3]) + struct.pack('<fB', value, 0),
                         bytes([13, 3]) + struct.pack('<fB', value, result)]
            add({'op': 'sync_write', 'group': 'pidf', 'slot': 0, 'key': 'p', 'value': value,
                 'verify': verify, 'responses': [list(r) for r in responses]},
                lambda v=value, r=responses, verify=verify: protocol.write_parameter(
                    Bus(r), protocol['pidf'][0]['p'], v, verify=verify, timeout=0.005))
add({'op': 'sync_write', 'group': 'pidf', 'slot': 0, 'key': 'p', 'value': 1, 'responses': []},
    lambda: protocol.write_parameter(Bus([]), protocol['pidf'][0]['p'], 1, timeout=0.005))
for text in ('maxmotion', 'encoder', 'PARAMETER', 'never_exists'):
    add({'op': 'find', 'text': text}, lambda text=text: sorted(protocol.frames.find(text)))
layout = json.loads(json.dumps(ref.DEFAULT_PARAMETER_LAYOUT))
layout['pidf']['p']['base_id'] = 33
add({'op': 'custom', 'layout': layout, 'slot_count': 2},
    lambda: ref.SparkMAXMotionProtocol(args.spec, parameter_layout=layout, slot_count=2).describe())
for slot, units in ((-1, 0), (4, 0), (0, 2)):
    add({'op': 'setpoint', 'slot': slot, 'units': units, 'setpoint': 1},
        lambda s=slot, u=units: protocol.maxmotion_setpoint_packet(1, slot=s, arbitrary_feedforward_units=u))

run = subprocess.run([str(args.oracle.resolve()), str(args.spec.resolve())],
    input=''.join(json.dumps(q) + '\n' for q in queries), text=True, capture_output=True, check=True)
actual = [json.loads(line) for line in run.stdout.splitlines()]
if len(actual) != len(expected):
    raise AssertionError(f'Expected {len(expected)} replies, received {len(actual)}')
errors = []
def equal(a, b):
    if isinstance(a, float) and isinstance(b, (float, int)):
        return math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12)
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(equal(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    return a == b
for i, (wanted, got) in enumerate(zip(expected, actual)):
    if wanted['ok'] != got['ok'] or (wanted['ok'] and not equal(wanted['value'], got.get('value'))):
        errors.append((i, queries[i], wanted, got))
for error in errors[:10]: print(json.dumps(error, indent=2))
print(f'{len(queries) - len(errors)}/{len(queries)} cases match; {len(protocol.frames)} JSON frames covered.')
sys.exit(bool(errors))
