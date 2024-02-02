cmake_minimum_required(VERSION 2.8.3)
project(spirecv_ros)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  actionlib_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  spirecv_msgs
  image_transport
  cv_bridge
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  std_msgs
)

find_package(SpireCV REQUIRED)
find_package(OpenCV 4 REQUIRED)

message(STATUS "SpireCV library status:")
message(STATUS "    include path: ${SV_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SV_INCLUDE_DIRS}
)


add_executable(web_cam
  samples/web_cam.cpp
)
target_link_libraries(web_cam ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(rtsp2topic
  samples/rtsp2topic.cpp
)
target_link_libraries(rtsp2topic ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(rtsp_forward
  samples/rtsp_forward.cpp
)
target_link_libraries(rtsp_forward ${catkin_LIBRARIES} ${SV_LIBRARIES})
