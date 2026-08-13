# claude: ROS2 port — catkin build replaced by ament_cmake.
#         Vendor tools not yet ported (still ROS1, excluded from build):
#           src/dump_point_node.cpp, src/cloud_process.cpp, src/subscriber.cpp
cmake_minimum_required(VERSION 3.8)
project(surestar_rfans_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Packet.msg
  msg/RfansPacket.msg
  msg/RfansScan.msg
  srv/RfansCommand.srv
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(src)

# common decode/IO core shared by both executables
add_library(rfans_core STATIC
  src/ioapi.cpp
  src/ssFrameLib.cpp
  src/bufferDecode.cpp
  src/calculation.cpp
)
ament_target_dependencies(rfans_core rclcpp std_msgs sensor_msgs builtin_interfaces)
target_link_libraries(rfans_core "${cpp_typesupport_target}" pcap)

add_executable(driver_node
  src/publisher.cpp
  src/rfans_driver.cpp
)
ament_target_dependencies(driver_node rclcpp std_msgs sensor_msgs builtin_interfaces)
target_link_libraries(driver_node rfans_core "${cpp_typesupport_target}")

add_executable(calculation_node
  src/cloud_node.cpp
)
ament_target_dependencies(calculation_node rclcpp std_msgs sensor_msgs builtin_interfaces)
target_link_libraries(calculation_node rfans_core "${cpp_typesupport_target}")

install(TARGETS driver_node calculation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
