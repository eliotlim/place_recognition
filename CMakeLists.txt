cmake_minimum_required(VERSION 2.8.3)
project(place_recognition)
include(ExternalProject)


set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(DEPENDENCY_DIR ${CMAKE_CURRENT_BINARY_DIR}/dependencies)
set(DEPENDENCY_INSTALL_DIR ${DEPENDENCY_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(DLib QUIET
  PATHS ${DEPENDENCY_INSTALL_DIR})
if(${DLib_FOUND})
  message("DLib library found, using it from the system")
  include_directories(${DLib_INCLUDE_DIRS})
  add_custom_target(DLib_Dependencies)
else(${DLib_FOUND})
  message("DLib library not found in the system, it will be downloaded on build")
  option(DOWNLOAD_DLib_dependency "Download DLib dependency" ON)
  if(${DOWNLOAD_DLib_dependency})
    ExternalProject_Add(DLib
      PREFIX ${DEPENDENCY_DIR}
      GIT_REPOSITORY http://github.com/dorian3d/DLib
      GIT_TAG v1.1-nonfree
      INSTALL_DIR ${DEPENDENCY_INSTALL_DIR}
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>)
    add_custom_target(DLib_LIBS ${CMAKE_COMMAND} ${CMAKE_SOURCE_DIR} DEPENDS DLib)
    include_directories(${DEPENDENCY_DIR}/src/DLib/include)
  else()
    message(SEND_ERROR "Please, activate DOWNLOAD_DLib_dependency option or download manually")
  endif(${DOWNLOAD_DLib_dependency})
endif(${DLib_FOUND})

find_package(DBoW2 QUIET
  PATHS ${DEPENDENCY_INSTALL_DIR})
if(${DBoW2_FOUND})
  message("DBoW2 library found, using it from the system")
  include_directories(${DBoW2_INCLUDE_DIRS})
  add_custom_target(DBoW2_Dependencies)
else(${DBoW2_FOUND})
  message("DBoW2 library not found in the system, it will be downloaded on build")
  option(DOWNLOAD_DBoW2_dependency "Download DBoW2 dependency" ON)
  if(${DOWNLOAD_DBoW2_dependency})
    ExternalProject_Add(DBoW2
      PREFIX ${DEPENDENCY_DIR}
      CONFIGURE_COMMAND "cmake" "." "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
      BUILD_COMMAND "make"
      BUILD_IN_SOURCE 1
      GIT_REPOSITORY http://github.com/dorian3d/DBoW2
      GIT_TAG v1.1-nonfree
      INSTALL_COMMAND "make" "install"
      INSTALL_DIR ${DEPENDENCY_INSTALL_DIR}
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>)
    add_custom_target(DBoW2_LIBS ${CMAKE_COMMAND} ${CMAKE_SOURCE_DIR} DEPENDS DLib)
    include_directories(${DBoW2_INCLUDE_DIRS})
    include_directories(${DEPENDENCY_DIR}/src/DBoW2/include)
  else()
    message(SEND_ERROR "Please, activate DOWNLOAD_DBoW2_dependency option or download manually")
  endif(${DOWNLOAD_DBoW2_dependency})
endif(${DBoW2_FOUND})

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

set(PROJECT_LIBRARIES
src/PlaceRecognizer.cpp
)

add_library(${PROJECT_NAME} SHARED ${PROJECT_LIBRARIES})

add_executable(place_recognizer src/place_recognition.cpp)
add_dependencies(place_recognizer
sensor_msgs
cv_bridge
image_transport
DBoW2_LIBS
DLib_LIBS)

target_link_libraries(place_recognizer
${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
${DLib_LIBS}
${DBoW2_LIBS}
)
