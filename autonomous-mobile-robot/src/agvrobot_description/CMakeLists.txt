cmake_minimum_required(VERSION 3.5)
project(agvrobot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the URDF, meshes, and launch files
install(
  DIRECTORY urdf launch meshes worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
