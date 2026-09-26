cmake_minimum_required(VERSION 3.16)
project(movemaster_driver VERSION 0.1.0 LANGUAGES CXX)

# MoveMasterDriver: manejo de los SPARK MAX del Movemaster (Position Control).
# Sin dependencias de ROS; se usa desde el plugin movemaster_hardware.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)
find_package(Threads REQUIRED)
if(NOT TARGET sparkmax_protocol::sparkmax_protocol)
  find_package(sparkmax_protocol REQUIRED)
endif()

add_library(movemaster_driver SHARED src/movemaster_driver.cpp)
add_library(movemaster_driver::movemaster_driver ALIAS movemaster_driver)
target_include_directories(movemaster_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(movemaster_driver PUBLIC
  sparkmax_protocol::sparkmax_protocol
  sparkmax_protocol::sparkmax_socketcan)
target_link_libraries(movemaster_driver PRIVATE Threads::Threads)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS movemaster_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

option(BUILD_TESTING "Compilar pruebas" ON)
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_movemaster_driver test/test_driver.cpp)
  target_link_libraries(test_movemaster_driver movemaster_driver)
  target_compile_definitions(test_movemaster_driver PRIVATE
    SPARK_FRAMES_JSON="${sparkmax_protocol_FRAMES_JSON}")
  add_test(NAME test_movemaster_driver COMMAND test_movemaster_driver)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(sparkmax_protocol)
  ament_package()
endif()
