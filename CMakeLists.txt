cmake_minimum_required(VERSION 2.8.3)
project(kalibr_detector)

SET(CMAKE_BUILD_TYPE "Release") # Debug

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  rosbag
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  message_generation
  nodelet
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  INCLUDE_DIRS camera_models/include
  INCLUDE_DIRS ethz_apriltag
  LIBRARIES kalibrs
  CATKIN_DEPENDS  image_transport roscpp rosbag sensor_msgs geometry_msgs tf cv_bridge message_runtime nodelet std_msgs
  DEPENDS Eigen OpenCV
)

###########
## Build ##
###########

include_directories(include
        ethz_apriltag
        camera_models/include
        )
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE c_files
        ${PROJECT_SOURCE_DIR}/ethz_apriltag/*.cc
        ${PROJECT_SOURCE_DIR}/camera_models/src/*.cc
        ${PROJECT_SOURCE_DIR}/src/*.cpp
        )
add_library(kalibrs ${c_files})

target_link_libraries(kalibrs
        ${catkin_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
)

add_executable(kalibr_detect_node main/kalibratag_detector_node.cpp)
target_link_libraries(kalibr_detect_node
        ${catkin_LIBRARIES}
        kalibrs
)
