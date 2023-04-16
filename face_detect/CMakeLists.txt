cmake_minimum_required(VERSION 2.8.3)
project(face_detect)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

#OPEN CV libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/share/opencv4/haarcascades/)
link_directories(${OpenCV_LIBRARY_DIRS})
set (LIBS ${LIBS} ${OpenCV_LIBS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uv_face
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(face_node src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(face_node
  ${catkin_LIBRARIES} ${LIBS}
)
