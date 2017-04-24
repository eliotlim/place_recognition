cmake_minimum_required(VERSION 2.8.3)
project(place_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs
)

include_directories(
  ${CATKIN_INCLUDE_DIRS}
  include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/PlaceRecognizer.cpp
)

add_executable(place_recognizer src/place_recognition.cpp)
target_link_libraries(place_recognizer ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(place_recognizer sensor_msgs cv_bridge image_transport)
