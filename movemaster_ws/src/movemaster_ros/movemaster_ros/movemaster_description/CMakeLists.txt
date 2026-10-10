cmake_minimum_required(VERSION 3.16)
project(movemaster_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description test/test_description.py)
endif()

ament_package()
