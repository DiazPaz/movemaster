cmake_minimum_required(VERSION 3.16)
project(movemaster_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch systemd DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bringup test/test_bringup.py)
endif()

ament_package()
