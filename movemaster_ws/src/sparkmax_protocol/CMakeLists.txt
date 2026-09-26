cmake_minimum_required(VERSION 3.16)
project(sparkmax_protocol VERSION 0.1.0 LANGUAGES CXX)

# Librería sin dependencias de ROS: se compila con colcon (ament_cmake) o como
# subproyecto CMake normal (add_subdirectory), p.ej. para pruebas en la Raspberry.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)

set(SPARKMAX_FRAMES_JSON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/spec/spark-frames-2.1.0")

# --- SparkMaxProtocol: parser JSON, FrameSpec, SignalCodec, CANPacket, parámetros
add_library(sparkmax_protocol SHARED
  src/json.cpp
  src/signal_codec.cpp
  src/frame_spec.cpp
  src/parameter.cpp
  src/sparkmax_protocol.cpp
  src/position_protocol.cpp
)
add_library(sparkmax_protocol::sparkmax_protocol ALIAS sparkmax_protocol)
target_include_directories(sparkmax_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sparkmax_protocol PUBLIC cxx_std_17)

# --- Transporte SocketCAN (Linux)
add_library(sparkmax_socketcan SHARED src/socketcan.cpp)
add_library(sparkmax_protocol::sparkmax_socketcan ALIAS sparkmax_socketcan)
target_link_libraries(sparkmax_socketcan PUBLIC sparkmax_protocol)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS sparkmax_protocol sparkmax_socketcan
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES spec/spark-frames-2.1.0 DESTINATION share/${PROJECT_NAME}/spec)

# Ruta del JSON para proyectos que incluyen éste con add_subdirectory().
set(sparkmax_protocol_FRAMES_JSON "${SPARKMAX_FRAMES_JSON_FILE}"
  CACHE INTERNAL "JSON de tramas SPARK (árbol de fuentes)")

option(BUILD_TESTING "Compilar pruebas" ON)
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_sparkmax_protocol test/test_protocol.cpp)
  target_link_libraries(test_sparkmax_protocol sparkmax_protocol)
  target_compile_definitions(test_sparkmax_protocol PRIVATE
    SPARK_FRAMES_JSON="${SPARKMAX_FRAMES_JSON_FILE}")
  add_test(NAME test_sparkmax_protocol COMMAND test_sparkmax_protocol)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package(CONFIG_EXTRAS cmake/sparkmax_protocol-extras.cmake)
endif()
