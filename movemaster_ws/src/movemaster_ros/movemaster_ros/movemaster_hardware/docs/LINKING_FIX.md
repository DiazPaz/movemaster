# Corrección de `symbol lookup error` — versión 0.1.1

El usuario reportó errores al cargar `FrameSpec::decode_payload` y
`SparkMAXMotionProtocol::parameter_read_packet`; el oracle C++ terminaba con
código 127. Los ejecutables no llegaban a realizar las pruebas de protocolo.

Ambos símbolos existen en la librería compilada desde el paquete. Reprodujimos
exactamente los dos mensajes colocando en `LD_LIBRARY_PATH` una librería con
el mismo SONAME `libsparkmax_protocol.so`, pero sin esas dos exportaciones.
Esto confirma que una colisión de librerías puede producir el fallo observado.
No demuestra por sí solo qué archivo carga la Raspberry Pi: para saberlo se
requiere inspeccionar su cargador local.

## Diagnóstico de la compilación anterior

Desde la carpeta que contiene el `CMakeLists.txt`:

```bash
ldd ./build/protocol_demo
ldd ./build/driver_test
```

Revisar a qué ruta resuelven `libsparkmax_protocol.so` y `libmovemaster_driver.so`.
Si apuntan a otro `install/`, otro workspace o una instalación anterior, los
ejecutables podrían cargar código distinto del usado al compilar.
También puede haber archivos de builds mezclados; sin esa salida no se puede
identificar cuál de las situaciones ocurre en el equipo del usuario.

Una prueba temporal, sin cambiar permanentemente la sesión, es priorizar el
directorio de esta compilación:

```bash
LD_LIBRARY_PATH="$PWD/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ctest --test-dir build --output-on-failure
```

Linux busca `LD_LIBRARY_PATH` antes que `DT_RUNPATH` en el caso habitual de
estos ejecutables. El comando prueba la hipótesis de carga de librerías externas;
no arregla un binario o una librería del propio `build/` que ya estén desfasados.

## Corrección incluida

En `CMakeLists.txt`, las dos librerías de la aplicación se declaran `STATIC`:

```cmake
add_library(sparkmax_protocol STATIC src/sparkmax_json_protocol.cpp)
add_library(movemaster_driver STATIC src/socketcan.cpp src/movemaster_driver.cpp src/driver_config.cpp)
set_target_properties(sparkmax_protocol movemaster_driver PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
```

El código se incorpora durante el enlace y los ejecutables no tienen esas dos
dependencias `.so`. Se mantiene `movemaster_hardware` como `SHARED`, porque lo
cargará pluginlib. PIC permite incorporar las librerías estáticas al plugin.
Las APIs, el JSON, los bytes CAN y la lógica del driver no cambian.

`tests/differential_test.py` ahora imprime el stderr de `protocol_oracle` al
fallar, de modo que el mensaje `undefined symbol` aparece directamente en CTest
en lugar de quedar oculto tras `CalledProcessError`.

No es necesario borrar el workspace, modificar `LD_LIBRARY_PATH` globalmente
ni eliminar librerías instaladas. Actualizar los archivos mencionados y crear
`build_fixed/` permite conservar la compilación anterior para comparar.

Fuentes:

- [Linux, ld.so: orden de búsqueda de librerías](https://man7.org/linux/man-pages/man8/ld.so.8.html).
- [CMake: POSITION_INDEPENDENT_CODE](https://cmake.org/cmake/help/latest/prop_tgt/POSITION_INDEPENDENT_CODE.html).
