cmake_minimum_required(VERSION 2.8.3)
project(dlut_viso2_ros)

find_package(catkin REQUIRED COMPONENTS 
  dlut_libvo 
  roscpp 
  sensor_msgs 
  nav_msgs 
  message_filters 
  image_transport 
  cv_bridge 
  image_geometry 
  tf 
  pcl_ros 
  std_srvs
  message_generation
)
find_package(cmake_modules REQUIRED)

find_package(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files( DIRECTORY msg FILES VisoInfo.msg)

generate_messages(
  DEPENDENCIES 
    std_msgs
)

catkin_package()

add_definitions(-msse3)

include_directories(src ${dlut_libvo_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(stereo_odometer
  src/stereo_odometer.cpp)
add_dependencies(stereo_odometer ${catkin_EXPORTED_TARGETS})
add_dependencies(stereo_odometer ${PROJECT_NAME}_gencpp)

add_executable(mono_odometer
  src/mono_odometer.cpp)
add_dependencies(mono_odometer ${catkin_EXPORTED_TARGETS})
add_dependencies(mono_odometer ${PROJECT_NAME}_gencpp)

target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${CERES_LIBRARIES})



