# movemaster_driver

`MoveMasterDriver`: maneja los SPARK MAX del Movemaster en **Position Control** sobre un
bus SocketCAN (hasta 6 controladores; la configuración por defecto contempla **3**).

- `movemaster_config.hpp` — valores dentro del código: número de ejes, IDs CAN,
  **relación de cada reductor** (`kGearRatioJoint1..3`), sentido, offsets, límites y PIDF.
- `gear_transmission.hpp` — conversión paramétrica rad ↔ rotaciones y rad/s ↔ RPM.
- `movemaster_driver.hpp` — ciclo de vida, hilo RX, confirmación de parámetros,
  referencias, heartbeat y watchdog.

```cpp
auto cfg = movemaster_driver::makeDefaultConfig("<share>/sparkmax_protocol/spec/spark-frames-2.1.0");
cfg.joints[0].transmission.gear_ratio = 100.0;       // o editar movemaster_config.hpp

movemaster_driver::MoveMasterDriver driver(cfg);
driver.open();
driver.configure();                  // bloqueante; confirma cada PARAMETER_WRITE
driver.waitForFeedback(std::chrono::seconds(2));
driver.arm();                        // referencia = posición medida

// en cada ciclo (≈ 50 Hz):
driver.setReference(0, 0.5);         // rad de la articulación
driver.sendReferences();             // POSITION_SETPOINT ×3 + heartbeat
auto s = driver.state(0);            // s.position_rad, s.velocity_rad_s, s.current_a

driver.disarm();
driver.close();
```

`examples/driver_demo.cpp` (`movemaster_driver_demo`) hace lo mismo desde la terminal,
con una rampa en el host. Las pruebas (`test/test_driver.cpp`) usan un bus simulado de
3 SPARK MAX (`test/fake_spark_bus.hpp`) y cubren la secuencia de configuración, la
conversión con reductor, límites, watchdog, rechazos, timeouts, tramas de error y
persistencia; pasan con AddressSanitizer, UBSan y ThreadSanitizer.
