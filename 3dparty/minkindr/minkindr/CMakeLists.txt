cmake_minimum_required(VERSION 3.3.2)
project(minkindr)

# Use ament_cmake for ROS2 build system
find_package(ament_cmake REQUIRED)

# Dependencies
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)

# Testing
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

find_package(Eigen3 REQUIRED)
#find_package(kindr REQUIRED)

# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(${KINDR_INCLUDE_DIR})

# Add files for QtCreator (optional)
file(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

# Install and export package
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()


