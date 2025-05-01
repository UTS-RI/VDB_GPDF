cmake_minimum_required(VERSION 3.10)
project(vdb_gpdf_msgs)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

###########
# Export #
###########

cs_install()
cs_export()
