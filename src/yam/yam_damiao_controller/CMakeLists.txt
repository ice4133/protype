cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()
project(yam_damiao_controller)
#set(CMAKE_BUILD_TYPE "RELEASEWITHDEBUGINFO")
set(CMAKE_BUILD_TYPE "RELEASE")
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yam_arm_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
pkg_search_module(TINYXML2 REQUIRED tinyxml2)

include_directories(include)

# 添加可执行文件
add_executable(YamController src/YamController.cpp
  src/mathematical_model/modern_robotics.cpp
  src/mathematical_model/ARX_R5_fun.cpp
  src/mathematical_model/yam_fun.cpp
  src/Damiao_6dof_node.cpp
  src/drivers/motor_control.cpp
)

target_include_directories(YamController PUBLIC
  ${LIBSERIAL_INCLUDE_DIR}
)

target_link_libraries(YamController ${DEPEND_LIB} 
  ${TINYXML2_LIBRARIES} 
  pinocchio::pinocchio
  Eigen3::Eigen
)

ament_target_dependencies(YamController 
rclcpp 
ament_index_cpp 
std_msgs 
yam_arm_msg
tf2 
tf2_msgs 
sensor_msgs 
tf2_ros 
geometry_msgs
)

install(TARGETS
  YamController
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${DEPEND_LIB}
  DESTINATION lib/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.yml"
  PATTERN "*.xml"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
  PATTERN "*.py"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
