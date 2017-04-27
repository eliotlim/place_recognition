cmake_minimum_required(VERSION 2.8.3)
project(place_recognition)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

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
  ThirdParty/DBoW2
)

set(PROJECT_LIBRARIES
src/PlaceRecognizer.cpp
)

add_library(${PROJECT_NAME} SHARED ${PROJECT_LIBRARIES})

add_executable(place_recognizer src/place_recognition.cpp)
add_dependencies(place_recognizer
sensor_msgs
cv_bridge
image_transport)

target_link_libraries(place_recognizer
${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${PROJECT_SOURCE_DIR}/ThirdParty/DBoW2/lib/libDBoW2.so
)
