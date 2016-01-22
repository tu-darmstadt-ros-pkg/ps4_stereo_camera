cmake_minimum_required(VERSION 2.8.3)
project(ps4_stereo_camera)

#SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ps4_stereo_camera
#  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport nodelet roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ps4_stereo_camera src/libcam.cpp)
target_link_libraries(ps4_stereo_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(pseye_camera ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(ps4_stereo_camera_node src/node.cpp)
target_link_libraries(ps4_stereo_camera_nodelet ps4_stereo_camera)

add_library(ps4_stereo_camera_nodelet src/nodelet.cpp)
target_link_libraries(ps4_stereo_camera_nodelet ps4_stereo_camera)


