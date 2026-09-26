cmake_minimum_required(VERSION 3.16)
project(movemaster_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json REQUIRED)

# -----------------------------------------------------------------------------
# SparkMaxProtocol: JSON REV -> tramas CAN (sin dependencias de ROS)
# -----------------------------------------------------------------------------
add_library(sparkmax_protocol SHARED
  src/sparkmax_protocol/can_packet.cpp
  src/sparkmax_protocol/signal_codec.cpp
  src/sparkmax_protocol/frame_spec.cpp
  src/sparkmax_protocol/parameter.cpp
  src/sparkmax_protocol/sparkmax_motion_protocol.cpp
  src/sparkmax_protocol/can_bus.cpp
)
target_include_directories(sparkmax_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(sparkmax_protocol PUBLIC nlohmann_json::nlohmann_json)

# -----------------------------------------------------------------------------
# MoveMasterDriver: 3 SPARK MAX, rad <-> rotaciones, estado y referencias
# -----------------------------------------------------------------------------
add_library(movemaster_driver SHARED
  src/movemaster_driver/movemaster_driver.cpp
)
target_link_libraries(movemaster_driver PUBLIC sparkmax_protocol)

# -----------------------------------------------------------------------------
# MovemasterHardware: plugin ros2_control
# -----------------------------------------------------------------------------
add_library(movemaster_hardware SHARED
  src/movemaster_hardware/movemaster_hardware.cpp
)
target_link_libraries(movemaster_hardware PUBLIC
  movemaster_driver
  ament_index_cpp::ament_index_cpp
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface movemaster_hardware.xml)

# -----------------------------------------------------------------------------
# Instalación
# -----------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})
install(
  TARGETS sparkmax_protocol movemaster_driver movemaster_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(SPARK_FRAMES_JSON "${CMAKE_CURRENT_SOURCE_DIR}/config/spark-frames-2.1.0.json")

  ament_add_gtest(test_sparkmax_protocol test/test_sparkmax_protocol.cpp)
  target_link_libraries(test_sparkmax_protocol sparkmax_protocol)
  target_compile_definitions(test_sparkmax_protocol PRIVATE
    SPARK_FRAMES_JSON="${SPARK_FRAMES_JSON}")

  ament_add_gtest(test_movemaster_driver test/test_movemaster_driver.cpp)
  target_link_libraries(test_movemaster_driver movemaster_driver)
  target_compile_definitions(test_movemaster_driver PRIVATE
    SPARK_FRAMES_JSON="${SPARK_FRAMES_JSON}")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp hardware_interface pluginlib rclcpp rclcpp_lifecycle nlohmann_json)
ament_package()
