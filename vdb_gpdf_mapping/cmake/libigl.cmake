if(TARGET igl::core)
  return()
endif()

include(FetchContent)
FetchContent_Declare(
libigl
GIT_REPOSITORY https://gitee.com/kin_zhang/libigl.git
GIT_TAG v2.4.0
)
FetchContent_MakeAvailable(libigl)