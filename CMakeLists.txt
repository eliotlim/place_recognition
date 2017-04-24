cmake_minimum_required(VERSION 2.8.3)
project(place_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs image_transport
)

include_directories(
  ${CATKIN_INCLUDE_DIRS}
)

add_executable(place_recognition src/place_recognition.cpp)
target_link_libraries(place_recognition ${catkin_LIBRARIES})
add_dependencies(place_recognition sensor_msgs cv_bridge image_transport)
