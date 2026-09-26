# sparkmax_protocol

Port en C++17 de `main/sparkmax_json_protocol.py`, sin dependencias externas ni de ROS.
Toda la geometría de las tramas (IDs, longitudes, posiciones de bits, tipos, escalas)
se lee de `spec/spark-frames-2.1.0` (JSON de REV); no se duplica en el código.

| Componente | Archivo | Equivalente Python |
|---|---|---|
| Parser JSON (orden de claves, enteros de 64 bits, independiente del locale) | `json.hpp` | `json.load` |
| `CANPacket` | `can_packet.hpp` | `CANPacket` |
| `SignalSpec`, `SignalValue`, `SignalCodec` | `signal_codec.hpp` | `SignalCodec` |
| `FrameSpec`, `SparkFrameDatabase`, `DecodedFrame` | `frame_spec.hpp` | `FrameSpec`, `SparkFrameDatabase` |
| `ParameterDefinition`, `ParameterGroup`, catálogos | `parameter.hpp` | `ParameterDefinition`, `ParameterGroup`, `DEFAULT_PARAMETER_LAYOUT` |
| `SparkMaxProtocol` (base), `MAXMotionProtocol` | `sparkmax_protocol.hpp` | `SparkMAXMotionProtocol` |
| **`PositionProtocol`** (sólo Position Control) | `position_protocol.hpp` | — (nuevo) |
| `CanTransport`, `SocketCanTransport` | `socketcan.hpp` | `can.Bus(interface="socketcan")` |

```cpp
#include "sparkmax_protocol/position_protocol.hpp"
using namespace sparkmax_protocol;

auto frames = SparkFrameDatabase::loadFile("spec/spark-frames-2.1.0");
PositionProtocol spark(frames, /*device_id=*/1);

CANPacket kp = spark.parameterWritePacket(spark.pidf(0)["p"], 1.0);   // PARAMETER_WRITE
CANPacket sp = spark.positionSetpointPacket(2.5 /*rot motor*/, /*slot=*/0);
Status2 s2   = spark.decodeStatus2(rx);   // posición [rot] y velocidad [RPM]

SocketCanTransport bus("can0");
bus.open({});
bus.send(sp, std::chrono::milliseconds(5));
```

Diferencias respecto a Python:

- Las señales constantes del JSON (`MAGIC_NUMBER`, `DATA_TYPE`) se rellenan solas.
- `positionSetpointPacket()` no asigna memoria: se puede llamar desde el lazo RT.
- El catálogo `pidf` añade `i_zone`, `d_filter`, `output_min`, `output_max` (IDs 17–20 de
  SparkParameters, **sin validar en banco**) y el grupo `setup` con los IDs que usa
  `teach_pendant_backend.py`.
- Los helpers síncronos `write_parameter()`/`configure_slot()` no se portaron: la espera
  de respuestas la hace `MoveMasterDriver` con un único hilo receptor.

Pruebas (`test/test_protocol.cpp`): los bytes esperados se generaron con la versión
Python; además se comparó la decodificación de 6540 cargas aleatorias sobre todas las
tramas del JSON con resultados idénticos.
