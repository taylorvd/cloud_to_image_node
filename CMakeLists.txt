cmake_minimum_required(VERSION 2.8.12)
project(cloud_to_image_node)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)


## Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs cv_bridge image_transport
)

## Build node
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cloud_to_image_node cloud_to_image_node.cpp)

target_link_libraries(cloud_to_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
