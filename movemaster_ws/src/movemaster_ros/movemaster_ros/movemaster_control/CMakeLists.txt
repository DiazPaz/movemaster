cmake_minimum_required(VERSION 3.16)
project(movemaster_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(controllers test/test_controllers.py)
endif()

ament_package()
