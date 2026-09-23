"""sparkmax_json_protocol.py

Parser/codec for REV SPARK CAN-frame JSON specs, specialized at the high level
for MAXMotion Position Control.

Design goals
------------
1. Frame arbitration IDs, lengths, signal positions, types, endianness and scales
   come from the REV JSON file rather than being duplicated in Python.
2. PIDF/MAXMotion parameter addressing is represented as a configurable nested
   dictionary (parameter IDs are a separate REV parameter specification and are
   therefore injected as a parameter layout).
3. Only MAXMotion Position Control is exposed at the high-level control API.
4. The low-level frame parser remains generic enough to inspect/encode/decode
   any little-endian frame in the JSON.

This module does not own the heartbeat thread. Keep using the heartbeat mechanism
already validated in your application.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, Mapping, MutableMapping, Optional
import copy
import json
import math
import struct
import time


# -----------------------------------------------------------------------------
# Parameter catalog
# -----------------------------------------------------------------------------
# IMPORTANT:
# The CAN-frame JSON describes HOW parameter access works (PARAMETER_WRITE,
# PARAMETER_WRITE_RESPONSE, READ_PARAMETER_x_AND_y, etc.), but it does not map
# semantic names such as "P 0" or "MAXMotion Max Accel 0" to parameter IDs.
# Those IDs belong to REV's separate SPARK parameter specification.
#
# The layout is intentionally data-driven and replaceable via the constructor.
# "base_id" is the slot-0 parameter ID and "slot_stride" advances to slot 1..3.

DEFAULT_PARAMETER_LAYOUT: Dict[str, Dict[str, Dict[str, Any]]] = {
    "pidf": {
        "p": {
            "base_id": 13,
            "slot_stride": 8,
            "type": "float",
            "description": "Proportional gain",
        },
        "i": {
            "base_id": 14,
            "slot_stride": 8,
            "type": "float",
            "description": "Integral gain",
        },
        "d": {
            "base_id": 15,
            "slot_stride": 8,
            "type": "float",
            "description": "Derivative gain",
        },
        "f": {
            "base_id": 16,
            "slot_stride": 8,
            "type": "float",
            "description": "Feedforward gain (F parameter)",
        },
    },
    "maxmotion": {
        "cruise_velocity": {
            "base_id": 166,
            "slot_stride": 5,
            "type": "float",
            "description": "MAXMotion cruise/max velocity",
            "unit": "RPM by default",
            "aliases": ["cruisevelocity", "max_velocity", "maxvelocity"],
        },
        "max_acceleration": {
            "base_id": 167,
            "slot_stride": 5,
            "type": "float",
            "description": "MAXMotion maximum acceleration",
            "unit": "RPM/s by default",
            "aliases": ["maxaccel", "max_accel", "maxacceleration"],
        },
        "allowed_profile_error": {
            "base_id": 169,
            "slot_stride": 5,
            "type": "float",
            "description": "MAXMotion allowed profile/closed-loop error",
            "unit": "rotations by default",
            "aliases": [
                "allowedprofileerror",
                "allowed_error",
                "allowed_closed_loop_error",
            ],
        },
    },
}


PARAMETER_TYPE_CODE = {
    "int": 1,
    "uint": 2,
    "float": 3,
    "bool": 4,
    "boolean": 4,
}

PARAMETER_TYPE_NAME = {value: key for key, value in PARAMETER_TYPE_CODE.items()}
PARAMETER_TYPE_NAME[4] = "bool"


# -----------------------------------------------------------------------------
# Small transport-neutral CAN packet
# -----------------------------------------------------------------------------

@dataclass(frozen=True)
class CANPacket:
    arbitration_id: int
    data: bytes = b""
    is_extended_id: bool = True
    is_remote_frame: bool = False
    dlc: Optional[int] = None
    frame_name: Optional[str] = None

    def to_python_can(self):
        """Convert to python-can's can.Message lazily."""
        try:
            import can  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "python-can is not installed. Install it or use CANPacket directly."
            ) from exc

        kwargs = {
            "arbitration_id": self.arbitration_id,
            "data": self.data,
            "is_extended_id": self.is_extended_id,
            "is_remote_frame": self.is_remote_frame,
        }
        if self.dlc is not None:
            kwargs["dlc"] = self.dlc
        return can.Message(**kwargs)

    def __repr__(self) -> str:
        data = self.data.hex(" ").upper() if self.data else "<RTR/no data>"
        return (
            f"CANPacket(frame={self.frame_name!r}, "
            f"id=0x{self.arbitration_id:08X}, dlc={self.dlc}, data={data})"
        )


# -----------------------------------------------------------------------------
# JSON signal/frame codec
# -----------------------------------------------------------------------------

class SpecError(RuntimeError):
    pass


class SignalCodec:
    """Encode/decode one signal using its JSON definition."""

    @staticmethod
    def _require_little_endian(signal_spec: Mapping[str, Any]) -> None:
        if signal_spec.get("isBigEndian", False):
            raise NotImplementedError(
                "This codec currently supports the little-endian REV signals used "
                "by MAXMotion/parameter access."
            )

    @staticmethod
    def _float_to_u32(value: float) -> int:
        return struct.unpack("<I", struct.pack("<f", float(value)))[0]

    @staticmethod
    def _u32_to_float(value: int) -> float:
        return struct.unpack("<f", struct.pack("<I", value & 0xFFFFFFFF))[0]

    @classmethod
    def encode_bits(cls, signal_spec: Mapping[str, Any], decoded_value: Any) -> int:
        cls._require_little_endian(signal_spec)

        signal_type = str(signal_spec["type"]).lower()
        bits = int(signal_spec["lengthBits"])
        scale = float(signal_spec.get("decodeScaleFactor", 1))
        offset = float(signal_spec.get("offset", 0))

        if signal_type == "float":
            raw_float = (float(decoded_value) - offset) / scale
            if bits != 32:
                raise SpecError(f"Unsupported float width: {bits}")
            return cls._float_to_u32(raw_float)

        if signal_type in ("boolean", "bool"):
            encoded = 1 if bool(decoded_value) else 0
        else:
            if scale == 0:
                raise SpecError("decodeScaleFactor cannot be zero")
            encoded = int(round((float(decoded_value) - offset) / scale))

        encoded_min = signal_spec.get("encodedMin")
        encoded_max = signal_spec.get("encodedMax")
        if encoded_min is not None and encoded < int(encoded_min):
            raise ValueError(
                f"Encoded value {encoded} is below encodedMin={encoded_min}"
            )
        if encoded_max is not None and encoded > int(encoded_max):
            raise ValueError(
                f"Encoded value {encoded} is above encodedMax={encoded_max}"
            )

        if signal_type == "int":
            min_signed = -(1 << (bits - 1))
            max_signed = (1 << (bits - 1)) - 1
            if not min_signed <= encoded <= max_signed:
                raise ValueError(
                    f"Signed {bits}-bit value out of range: {encoded}"
                )
            if encoded < 0:
                encoded = (1 << bits) + encoded
        elif signal_type in ("uint", "boolean", "bool"):
            if not 0 <= encoded < (1 << bits):
                raise ValueError(
                    f"Unsigned {bits}-bit value out of range: {encoded}"
                )
        else:
            raise SpecError(f"Unsupported signal type: {signal_type!r}")

        return encoded

    @classmethod
    def decode_bits(cls, signal_spec: Mapping[str, Any], raw_bits: int) -> Any:
        cls._require_little_endian(signal_spec)

        signal_type = str(signal_spec["type"]).lower()
        bits = int(signal_spec["lengthBits"])
        scale = float(signal_spec.get("decodeScaleFactor", 1))
        offset = float(signal_spec.get("offset", 0))

        raw_bits &= (1 << bits) - 1

        if signal_type == "float":
            if bits != 32:
                raise SpecError(f"Unsupported float width: {bits}")
            raw_float = cls._u32_to_float(raw_bits)
            return raw_float * scale + offset

        if signal_type == "int":
            sign_bit = 1 << (bits - 1)
            if raw_bits & sign_bit:
                encoded = raw_bits - (1 << bits)
            else:
                encoded = raw_bits
            return encoded * scale + offset

        if signal_type == "uint":
            return raw_bits * scale + offset

        if signal_type in ("boolean", "bool"):
            return bool(raw_bits)

        raise SpecError(f"Unsupported signal type: {signal_type!r}")


class FrameSpec(Mapping[str, Any]):
    """Dictionary-like wrapper around one frame entry in the JSON."""

    def __init__(self, key: str, section: str, spec: Mapping[str, Any]):
        self.key = key
        self.section = section
        self._spec = dict(spec)

    def __getitem__(self, key: str) -> Any:
        return self._spec[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._spec)

    def __len__(self) -> int:
        return len(self._spec)

    @property
    def base_arb_id(self) -> int:
        return int(self._spec["arbId"])

    @property
    def length_bytes(self) -> int:
        return int(self._spec.get("lengthBytes", 0))

    @property
    def signals(self) -> Mapping[str, Mapping[str, Any]]:
        return self._spec.get("signals", {})

    @property
    def rtr(self) -> bool:
        return bool(self._spec.get("rtr", False))

    def arbitration_id(self, device_id: int) -> int:
        if not 0 <= device_id <= 63:
            raise ValueError("SPARK CAN device ID must be between 0 and 63")
        # The JSON's arbId is defined with Device Number = 0.
        return (self.base_arb_id & ~0x3F) | device_id

    def encode_payload(
        self,
        values: Mapping[str, Any],
        *,
        require_all: bool = False,
    ) -> bytes:
        if self.rtr:
            return b""

        frame_bits = self.length_bytes * 8
        raw_frame = 0

        for signal_name, signal_spec in self.signals.items():
            if signal_name in values:
                decoded_value = values[signal_name]
            elif require_all:
                raise KeyError(
                    f"Missing signal {signal_name!r} for frame {self.key}"
                )
            else:
                # Reserved/config bits in REV commands are generally zero unless
                # explicitly supplied. This makes sparse dictionaries convenient.
                decoded_value = 0

            bit_position = int(signal_spec["bitPosition"])
            length_bits = int(signal_spec["lengthBits"])
            encoded = SignalCodec.encode_bits(signal_spec, decoded_value)
            mask = (1 << length_bits) - 1
            raw_frame |= (encoded & mask) << bit_position

        if raw_frame >= (1 << frame_bits):
            raise SpecError(f"Encoded frame {self.key} does not fit in {frame_bits} bits")

        return raw_frame.to_bytes(self.length_bytes, byteorder="little", signed=False)

    def decode_payload(self, data: bytes) -> Dict[str, Any]:
        if len(data) != self.length_bytes:
            raise ValueError(
                f"Frame {self.key} expects {self.length_bytes} bytes, got {len(data)}"
            )

        raw_frame = int.from_bytes(data, byteorder="little", signed=False)
        decoded: Dict[str, Any] = {}

        for signal_name, signal_spec in self.signals.items():
            bit_position = int(signal_spec["bitPosition"])
            length_bits = int(signal_spec["lengthBits"])
            mask = (1 << length_bits) - 1
            raw_bits = (raw_frame >> bit_position) & mask
            decoded[signal_name] = SignalCodec.decode_bits(signal_spec, raw_bits)

        return decoded

    def packet(
        self,
        device_id: int,
        values: Optional[Mapping[str, Any]] = None,
    ) -> CANPacket:
        return CANPacket(
            arbitration_id=self.arbitration_id(device_id),
            data=self.encode_payload(values or {}),
            is_extended_id=True,
            is_remote_frame=self.rtr,
            dlc=self.length_bytes,
            frame_name=self.key,
        )


class SparkFrameDatabase(Mapping[str, FrameSpec]):
    """Loads REV's SPARK frame JSON and offers dictionary-style frame access."""

    def __init__(self, json_path: str | Path):
        self.path = Path(json_path)
        with self.path.open("r", encoding="utf-8") as fh:
            self.raw: Dict[str, Any] = json.load(fh)

        self.frames: Dict[str, FrameSpec] = {}
        for section in ("periodicFrames", "nonPeriodicFrames"):
            for key, spec in self.raw.get(section, {}).items():
                if key in self.frames:
                    raise SpecError(f"Duplicate frame key: {key}")
                self.frames[key] = FrameSpec(key, section, spec)

    def __getitem__(self, key: str) -> FrameSpec:
        return self.frames[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self.frames)

    def __len__(self) -> int:
        return len(self.frames)

    @property
    def frames_version(self) -> str:
        return str(self.raw.get("framesVersion", "unknown"))

    @property
    def device_info(self) -> Mapping[str, Any]:
        return self.raw.get("deviceInfo", {})

    def find(self, text: str) -> Dict[str, FrameSpec]:
        needle = text.lower()
        return {
            key: frame
            for key, frame in self.frames.items()
            if needle in key.lower()
            or needle in str(frame.get("name", "")).lower()
            or needle in str(frame.get("description", "")).lower()
        }


# -----------------------------------------------------------------------------
# Parameter definitions / dictionary access
# -----------------------------------------------------------------------------

@dataclass(frozen=True)
class ParameterDefinition:
    group: str
    key: str
    slot: int
    parameter_id: int
    value_type: str
    description: str = ""
    unit: Optional[str] = None

    @property
    def pair_start_id(self) -> int:
        return self.parameter_id & ~1

    @property
    def pair_index(self) -> int:
        return self.parameter_id - self.pair_start_id

    @property
    def read_frame_name(self) -> str:
        return f"READ_PARAMETER_{self.pair_start_id}_AND_{self.pair_start_id + 1}"


class ParameterGroup(Mapping[str, ParameterDefinition]):
    """Dictionary-like group supporting canonical names and aliases."""

    def __init__(self, canonical: Mapping[str, ParameterDefinition], aliases: Mapping[str, str]):
        self._canonical = dict(canonical)
        self._aliases = dict(aliases)

    @staticmethod
    def _normalize(key: str) -> str:
        return key.strip().lower().replace("-", "_").replace(" ", "_")

    def __getitem__(self, key: str) -> ParameterDefinition:
        normalized = self._normalize(key)
        canonical = self._aliases.get(normalized, normalized)
        return self._canonical[canonical]

    def __iter__(self) -> Iterator[str]:
        return iter(self._canonical)

    def __len__(self) -> int:
        return len(self._canonical)

    def as_dict(self) -> Dict[str, ParameterDefinition]:
        return dict(self._canonical)


# -----------------------------------------------------------------------------
# MAXMotion Position high-level protocol
# -----------------------------------------------------------------------------

class SparkMAXMotionProtocol(Mapping[str, Any]):
    """
    MAXMotion Position-only protocol facade.

    Dictionary access:
        spark["pidf"][0]["p"]
        spark["pidf"][0]["f"]
        spark["maxmotion"][0]["max_acceleration"]
        spark["maxmotion"][0]["cruisevelocity"]     # alias
        spark["frames"]["MAXMOTION_POSITION_SETPOINT"]

    Frame construction:
        spark.parameter_write_packet(spark["pidf"][0]["p"], 0.05)
        spark.maxmotion_setpoint_packet(0.5, slot=0)
    """

    def __init__(
        self,
        json_path: str | Path,
        *,
        device_id: int = 1,
        parameter_layout: Optional[Mapping[str, Mapping[str, Mapping[str, Any]]]] = None,
        slot_count: int = 4,
    ):
        if not 0 <= device_id <= 63:
            raise ValueError("device_id must be between 0 and 63")
        if slot_count <= 0:
            raise ValueError("slot_count must be positive")

        self.device_id = int(device_id)
        self.slot_count = int(slot_count)
        self.frames = SparkFrameDatabase(json_path)
        self.parameter_layout = copy.deepcopy(
            parameter_layout if parameter_layout is not None else DEFAULT_PARAMETER_LAYOUT
        )

        # Required frame names. Fail early if the loaded spec is incompatible.
        for required in (
            "MAXMOTION_POSITION_SETPOINT",
            "PARAMETER_WRITE",
            "PARAMETER_WRITE_RESPONSE",
        ):
            if required not in self.frames:
                raise SpecError(
                    f"Loaded JSON does not contain required frame {required!r}"
                )

        self._pidf = self._build_group_by_slot("pidf")
        self._maxmotion = self._build_group_by_slot("maxmotion")

        self._access: Dict[str, Any] = {
            "pidf": self._pidf,
            "maxmotion": self._maxmotion,
            "frames": self.frames,
            "setpoint": self.frames["MAXMOTION_POSITION_SETPOINT"],
        }

    def __getitem__(self, key: str) -> Any:
        return self._access[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._access)

    def __len__(self) -> int:
        return len(self._access)

    def _build_group_by_slot(self, group: str) -> Dict[int, ParameterGroup]:
        layout = self.parameter_layout[group]
        result: Dict[int, ParameterGroup] = {}

        for slot in range(self.slot_count):
            canonical: Dict[str, ParameterDefinition] = {}
            aliases: Dict[str, str] = {}

            for key, meta in layout.items():
                parameter_id = int(meta["base_id"]) + slot * int(meta["slot_stride"])
                definition = ParameterDefinition(
                    group=group,
                    key=key,
                    slot=slot,
                    parameter_id=parameter_id,
                    value_type=str(meta.get("type", "float")).lower(),
                    description=str(meta.get("description", "")),
                    unit=meta.get("unit"),
                )
                canonical[key] = definition
                aliases[ParameterGroup._normalize(key)] = key
                for alias in meta.get("aliases", []):
                    aliases[ParameterGroup._normalize(str(alias))] = key

            result[slot] = ParameterGroup(canonical, aliases)

        return result

    # ---- parameter value packing -------------------------------------------------

    @staticmethod
    def pack_parameter_value(value: Any, value_type: str) -> int:
        value_type = value_type.lower()
        if value_type == "float":
            return struct.unpack("<I", struct.pack("<f", float(value)))[0]
        if value_type == "int":
            return int(value) & 0xFFFFFFFF
        if value_type == "uint":
            integer = int(value)
            if not 0 <= integer <= 0xFFFFFFFF:
                raise ValueError("uint parameter must fit in 32 bits")
            return integer
        if value_type in ("bool", "boolean"):
            return 1 if bool(value) else 0
        raise ValueError(f"Unsupported parameter type: {value_type!r}")

    @staticmethod
    def unpack_parameter_value(raw_u32: int, value_type: str) -> Any:
        value_type = value_type.lower()
        raw_u32 &= 0xFFFFFFFF
        if value_type == "float":
            return struct.unpack("<f", struct.pack("<I", raw_u32))[0]
        if value_type == "int":
            return raw_u32 - 0x100000000 if raw_u32 & 0x80000000 else raw_u32
        if value_type == "uint":
            return raw_u32
        if value_type in ("bool", "boolean"):
            return bool(raw_u32)
        raise ValueError(f"Unsupported parameter type: {value_type!r}")

    # ---- frame builders ----------------------------------------------------------

    def parameter_write_packet(
        self,
        parameter: ParameterDefinition,
        value: Any,
    ) -> CANPacket:
        frame = self.frames["PARAMETER_WRITE"]
        raw_value = self.pack_parameter_value(value, parameter.value_type)
        return frame.packet(
            self.device_id,
            {
                "PARAMETER_ID": parameter.parameter_id,
                # VALUE is uint in the frame JSON because the true type is
                # parameter-dependent. We intentionally supply the raw 32-bit bits.
                "VALUE": raw_value,
            },
        )

    def parameter_read_packet(self, parameter: ParameterDefinition) -> CANPacket:
        frame_name = parameter.read_frame_name
        if frame_name not in self.frames:
            raise SpecError(
                f"Loaded JSON has no read frame for parameter {parameter.parameter_id}: "
                f"{frame_name}"
            )
        return self.frames[frame_name].packet(self.device_id)

    def maxmotion_setpoint_packet(
        self,
        setpoint: float,
        *,
        slot: int = 0,
        arbitrary_feedforward: float = 0.0,
        arbitrary_feedforward_units: int = 0,
    ) -> CANPacket:
        if slot not in range(self.slot_count):
            raise ValueError(f"slot must be in 0..{self.slot_count - 1}")
        if arbitrary_feedforward_units not in (0, 1):
            raise ValueError("arbitrary_feedforward_units must be 0 (V) or 1 (duty cycle)")

        frame = self.frames["MAXMOTION_POSITION_SETPOINT"]
        return frame.packet(
            self.device_id,
            {
                "SETPOINT": float(setpoint),
                "ARBITRARY_FEEDFORWARD": float(arbitrary_feedforward),
                "PID_SLOT": slot,
                "ARBITRARY_FEEDFORWARD_UNITS": arbitrary_feedforward_units,
                "RESERVED": 0,
            },
        )

    # ---- decoders ----------------------------------------------------------------

    def decode_parameter_write_response(self, data: bytes) -> Dict[str, Any]:
        frame = self.frames["PARAMETER_WRITE_RESPONSE"]
        decoded = frame.decode_payload(data)

        parameter_id = int(decoded["PARAMETER_ID"])
        parameter_type_code = int(decoded["PARAMETER_TYPE"])
        raw_value = int(decoded["VALUE"])
        result_code = int(decoded["RESULT_CODE"])

        value_type = PARAMETER_TYPE_NAME.get(parameter_type_code, "unknown")
        current_value = (
            self.unpack_parameter_value(raw_value, value_type)
            if value_type != "unknown"
            else raw_value
        )

        return {
            "parameter_id": parameter_id,
            "parameter_type_code": parameter_type_code,
            "parameter_type": value_type,
            "current_value": current_value,
            "result_code": result_code,
            "success": result_code == 0,
        }

    def decode_parameter_read_response(
        self,
        parameter: ParameterDefinition,
        data: bytes,
    ) -> Any:
        frame = self.frames[parameter.read_frame_name]
        decoded = frame.decode_payload(data)
        field = (
            "FIRST_PARAMETER_VALUE"
            if parameter.pair_index == 0
            else "SECOND_PARAMETER_VALUE"
        )
        raw_u32 = int(decoded[field])
        return self.unpack_parameter_value(raw_u32, parameter.value_type)

    # ---- optional synchronous python-can helpers --------------------------------

    @staticmethod
    def _send_packet(bus: Any, packet: CANPacket) -> None:
        bus.send(packet.to_python_can())

    def write_parameter(
        self,
        bus: Any,
        parameter: ParameterDefinition,
        value: Any,
        *,
        timeout: float = 1.0,
        verify: bool = True,
    ) -> Dict[str, Any]:
        """
        Send PARAMETER_WRITE and wait synchronously for its response.

        NOTE: This consumes frames from bus.recv(). In a final threaded library,
        route responses through one central RX dispatcher instead of having multiple
        consumers call recv() concurrently.
        """
        packet = self.parameter_write_packet(parameter, value)
        self._send_packet(bus, packet)

        response_frame = self.frames["PARAMETER_WRITE_RESPONSE"]
        expected_id = response_frame.arbitration_id(self.device_id)
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            rx = bus.recv(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            if rx is None or not rx.is_extended_id:
                continue
            if rx.arbitration_id != expected_id:
                continue

            decoded = self.decode_parameter_write_response(bytes(rx.data))
            if decoded["parameter_id"] != parameter.parameter_id:
                continue

            decoded["requested_value"] = value
            decoded["parameter"] = parameter

            if verify and decoded["success"]:
                current = decoded["current_value"]
                if parameter.value_type == "float":
                    decoded["value_matches"] = math.isclose(
                        float(current), float(value), rel_tol=1e-5, abs_tol=1e-7
                    )
                else:
                    decoded["value_matches"] = current == value
            else:
                decoded["value_matches"] = None

            return decoded

        raise TimeoutError(
            f"Timeout waiting for PARAMETER_WRITE_RESPONSE for parameter "
            f"{parameter.parameter_id}"
        )

    def send_setpoint(self, bus: Any, setpoint: float, *, slot: int = 0, **kwargs: Any) -> CANPacket:
        packet = self.maxmotion_setpoint_packet(setpoint, slot=slot, **kwargs)
        self._send_packet(bus, packet)
        return packet

    # ---- convenient bulk configuration ------------------------------------------

    def configure_slot(
        self,
        bus: Any,
        *,
        slot: int = 0,
        pidf: Optional[Mapping[str, float]] = None,
        maxmotion: Optional[Mapping[str, float]] = None,
        timeout: float = 1.0,
    ) -> Dict[str, Dict[str, Any]]:
        """Write only the keys supplied in pidf/maxmotion and return responses."""
        if slot not in range(self.slot_count):
            raise ValueError(f"slot must be in 0..{self.slot_count - 1}")

        results: Dict[str, Dict[str, Any]] = {}

        for group_name, supplied in (("pidf", pidf), ("maxmotion", maxmotion)):
            if not supplied:
                continue
            group: ParameterGroup = self[group_name][slot]
            for key, value in supplied.items():
                parameter = group[key]
                results[f"{group_name}.{parameter.key}"] = self.write_parameter(
                    bus, parameter, value, timeout=timeout, verify=True
                )

        return results

    def describe(self) -> Dict[str, Any]:
        """Return a serializable summary useful for debugging/introspection."""
        def group_to_dict(group_by_slot: Mapping[int, ParameterGroup]) -> Dict[int, Any]:
            out: Dict[int, Any] = {}
            for slot, group in group_by_slot.items():
                out[slot] = {
                    key: {
                        "parameter_id": p.parameter_id,
                        "type": p.value_type,
                        "unit": p.unit,
                        "description": p.description,
                        "read_frame": p.read_frame_name,
                    }
                    for key, p in group.items()
                }
            return out

        setpoint_frame = self.frames["MAXMOTION_POSITION_SETPOINT"]
        return {
            "frames_version": self.frames.frames_version,
            "device_id": self.device_id,
            "device_info": dict(self.frames.device_info),
            "maxmotion_position_setpoint": {
                "base_arb_id": setpoint_frame.base_arb_id,
                "device_arb_id": setpoint_frame.arbitration_id(self.device_id),
                "length_bytes": setpoint_frame.length_bytes,
                "signals": dict(setpoint_frame.signals),
            },
            "pidf": group_to_dict(self._pidf),
            "maxmotion": group_to_dict(self._maxmotion),
        }
