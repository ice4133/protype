cmake_minimum_required(VERSION 3.8)
project(yam_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
