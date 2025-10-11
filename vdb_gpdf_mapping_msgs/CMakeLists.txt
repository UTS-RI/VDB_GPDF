cmake_minimum_required(VERSION 3.10)
project(vdb_gpdf_mapping_msgs)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Use ament_cmake for ROS2
find_package(ament_cmake REQUIRED)

# Specify dependencies for message generation (if needed)
find_package(std_msgs REQUIRED)

# If this package includes message definitions
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "srv/SliceQueryMap.srv"
  "srv/PointsQueryMap.srv"
  DEPENDENCIES std_msgs  # Add other dependencies if needed
)

# Export the package
ament_package()
