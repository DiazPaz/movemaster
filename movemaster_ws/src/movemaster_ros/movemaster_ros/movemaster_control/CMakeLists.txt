cmake_minimum_required(VERSION 3.16)
project(movemaster_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(robot_description test/test_robot_description.py)
endif()

ament_package()
