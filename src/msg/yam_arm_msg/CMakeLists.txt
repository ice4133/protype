cmake_minimum_required(VERSION 3.8)
project(yam_arm_msg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YamCmd.msg"
  "msg/YamStatus.msg"
  DEPENDENCIES 
  sensor_msgs
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
