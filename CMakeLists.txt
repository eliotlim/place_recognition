cmake_minimum_required(VERSION 2.8.3)
project(place_recognition)
include(ExternalProject)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(DBoW2 QUIET
  PATHS ${DEPENDENCY_INSTALL_DIR})
if(${DBoW2_FOUND})
  message("DLib library found, using it from the system")
  include_directories(${DBoW2_INCLUDE_DIRS})
  add_custom_target(Dependencies)
else(${DBoW2_FOUND})
  message("DBoW2 library not found in the system, it will be downloaded on build")
  option(DOWNLOAD_DBoW2_dependency "Download DLib dependency" ON)
  if(${DOWNLOAD_DBoW2_dependency})
    ExternalProject_Add(DBoW2
      PREFIX ${DEPENDENCY_DIR}
      GIT_REPOSITORY http://github.com/dorian3d/DBoW2
      GIT_TAG v1.1-nonfree
      INSTALL_DIR ${DEPENDENCY_INSTALL_DIR}
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>)
    add_custom_target(Dependencies ${CMAKE_COMMAND} ${CMAKE_SOURCE_DIR} DEPENDS DBoW2)
  else()
    message(SEND_ERROR "Please, activate DOWNLOAD_DLib_dependency option or download manually")
  endif(${DOWNLOAD_DBoW2_dependency})
endif(${DBoW2_FOUND})

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
