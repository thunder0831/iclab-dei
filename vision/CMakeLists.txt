cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  message_generation
  dynamic_reconfigure
  cv_bridge
)

add_message_files(
   FILES
   x264Packet.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vision
  CATKIN_DEPENDS roscpp message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# find ffmpeg libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(FFMPEG)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${FFMPEG_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(vision src/vision.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
target_link_libraries(vision ${catkin_LIBRARIES} ${FFMPEG_LIBRARIES})
