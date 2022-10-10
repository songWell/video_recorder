cmake_minimum_required(VERSION 2.8.3)
project(video_recorder)

set(cv_bridge_DIR /home/sfj/catkin_ws/install/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
        draco_point_cloud_transport
        rosbag
)

catkin_package(
#        LIBRARIES ${PROJECT_NAME}
#        CATKIN_DEPENDS  draco_point_cloud_transport rosbag
)



find_package(OpenCV 4.5.5 REQUIRED)
include_directories(include  ${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})

add_executable(video_recorder src/video_recorder.cpp)
target_link_libraries(video_recorder ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} ${rosbag_LIBRARIES})

add_executable(video_decorder src/video_decorder.cpp)
target_link_libraries(video_decorder ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})



