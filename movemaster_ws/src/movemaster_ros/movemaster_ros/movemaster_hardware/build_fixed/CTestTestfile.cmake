# CMake generated Testfile for 
# Source directory: /home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware
# Build directory: /home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/build_fixed
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(driver_fake_bus "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/build_fixed/driver_test" "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/spec/spark-frames-2.1.0")
set_tests_properties(driver_fake_bus PROPERTIES  _BACKTRACE_TRIPLES "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/CMakeLists.txt;47;add_test;/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/CMakeLists.txt;0;")
add_test(protocol_demo_no_can "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/build_fixed/protocol_demo" "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/spec/spark-frames-2.1.0")
set_tests_properties(protocol_demo_no_can PROPERTIES  _BACKTRACE_TRIPLES "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/CMakeLists.txt;48;add_test;/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/CMakeLists.txt;0;")
add_test(python_cpp_parity "/usr/bin/python3" "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/tests/differential_test.py" "--oracle" "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/build_fixed/protocol_oracle")
set_tests_properties(python_cpp_parity PROPERTIES  _BACKTRACE_TRIPLES "/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/CMakeLists.txt;51;add_test;/home/movemaster/movemaster/movemaster_ws/src/movemaster_ros/movemaster_ros/movemaster_hardware/CMakeLists.txt;0;")
