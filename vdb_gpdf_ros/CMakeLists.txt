cmake_minimum_required(VERSION 3.10)  # ensure that add_definition is used
project(vdb_gpdf_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             roscpp
             tf2_ros
             tf2_sensor_msgs
             std_msgs
             sensor_msgs
             message_generation
             pcl_ros
             vdb_gpdf_msgs
             vdb_gpdf)

catkin_package(
                CATKIN_DEPENDS
                roscpp
                rospy
                tf2_ros
                tf2
                std_msgs
                sensor_msgs
                message_runtime
                pcl_ros
                vdb_gpdf_msgs
                vdb_gpdf)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -O3 -fopenmp")
#set(DEIGEN_STACK_ALLOCATION_LIMIT "0")
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=10000000) # 1 MB

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/vdb_gpdf_mapper.cpp
        src/transform.cpp
        )

###############
# Executables #
###############

cs_add_executable(vdb_gpdf_mapping
        src/vdb_gpdf_mapper_node.cpp
        )

target_link_libraries(vdb_gpdf_mapping ${catkin_LIBRARIES} ${PROJECT_NAME}
        glog::glog
        OpenVDB::openvdb
        )
target_include_directories(vdb_gpdf_mapping PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
      )
        
# =========== test  ================== #
cs_add_executable(testing
        test/test_dda.cpp
        )
target_link_libraries(testing ${PROJECT_NAME} 
        glog::glog
        OpenVDB::openvdb
        )

###########
# Export #
###########

cs_install()
cs_export()
