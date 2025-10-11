cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(vdb_gpdf_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp -pthread")
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=10000000)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features)
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(minkindr)
find_package(pcl_conversions)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

# include
include_directories(
  #${CMAKE_SOURCE_DIR}/src/vdb_gpdf/include
  include
  /home/lan/Code/ros2_vdbgpdf/src/vdb_gpdf/include
  /home/lan/Code/ros2_vdbgpdf/src/vdb_gpdf_mapping/include
  /home/lan/Code/ros2_vdbgpdf/src/3dparty/minkindr/minkindr/include
  
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${vdb_gpdf_mapping_msgs_INCLUDE_DIRS}
)

# Custom packages
find_package(vdb_gpdf REQUIRED)
find_package(vdb_gpdf_mapping_msgs REQUIRED)

# Library
add_library(${PROJECT_NAME}
  src/vdb_gpdf_mapper.cpp
  src/transform_align.cpp
)

target_link_libraries(${PROJECT_NAME}
  vdb_gpdf::vdb_gpdf
  ${PCL_LIBRARIES}
  glog::glog
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  PCL
  glog
  Eigen3
  OpenVDB
  minkindr
  vdb_gpdf_mapping_msgs
  tf2_eigen
  pcl_conversions
)

# Executable
add_executable(vdb_gpdf_mapping_node vdb_gpdf_mapping_node.cpp)

target_link_libraries(vdb_gpdf_mapping_node ${PROJECT_NAME}
  vdb_gpdf::vdb_gpdf
  ${PCL_LIBRARIES}
  glog::glog
)

ament_target_dependencies(vdb_gpdf_mapping_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  PCL
  glog
  Eigen3
  OpenVDB
  vdb_gpdf_mapping_msgs
  tf2_eigen
  minkindr
)
        
# Tests
add_executable(testing test/test_dda.cpp)
target_link_libraries(testing ${PROJECT_NAME} vdb_gpdf::vdb_gpdf)
ament_target_dependencies(testing
  rclcpp
  glog
  OpenVDB
  
)

install(TARGETS
  ${PROJECT_NAME}
  vdb_gpdf_mapping_node
  testing
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()