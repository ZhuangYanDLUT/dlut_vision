# project
cmake_minimum_required (VERSION 2.6)
project (dlut_libvo)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

find_package(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# directories
set (DLUT_LIBVO_SRC_DIR src)

# include directory
include_directories("${DLUT_LIBVO_SRC_DIR}")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB DLUT_LIBVO_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_executable(dlut_libvo ${DLUT_LIBVO_SRC_FILES})
target_link_libraries (dlutvo png ${CERES_LIBRARIES})

