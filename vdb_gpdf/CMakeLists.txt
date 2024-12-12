cmake_minimum_required(VERSION 3.10)
project(vdb_gpdf)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             roscpp
             std_msgs
             sensor_msgs
             pcl_ros)

catkin_package(
                CATKIN_DEPENDS
                roscpp
                rospy
                std_msgs
                sensor_msgs
                pcl_ros
                )

find_package(catkin_simple REQUIRED)
find_package(Eigen3)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -O3 -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#set(DEIGEN_STACK_ALLOCATION_LIMIT "0")
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=10000000) # 1 MB
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/MarchingCubes.cpp
        src/VDBVolume.cpp
        )
###############
# Executables #
###############

target_link_libraries(${PROJECT_NAME} #${CERES_LIBRARIES} 
        OpenVDB::openvdb
        )

###########
# Export #
###########

cs_install()
cs_export(INCLUDE_DIRS)