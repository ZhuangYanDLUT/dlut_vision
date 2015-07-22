cmake_minimum_required(VERSION 2.8.3)
project(dlut_libvo)

find_package(catkin REQUIRED )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
#set(CMAKE_CXX_FLAGS "-msse3")
add_definitions(-msse3)
add_library(viso2
  dlut_libvo/src/filter.cpp
  dlut_libvo/src/matcher.cpp
  dlut_libvo/src/matrix.cpp
  dlut_libvo/src/reconstruction.cpp
  dlut_libvo/src/triangle.cpp
  dlut_libvo/src/viso.cpp
  dlut_libvo/src/viso_mono.cpp
  dlut_libvo/src/viso_stereo.cpp
  dlut_libvo/src/P3p.cpp)

include_directories(libviso2/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS dlut_libvo/src
  LIBRARIES viso2
)
