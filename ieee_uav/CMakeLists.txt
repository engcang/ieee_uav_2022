cmake_minimum_required(VERSION 2.8.12)
project(ieee_uav)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  yolo_ros_simple
  mavros
  gazebo_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
    odom_array.msg
)

generate_messages(
  DEPENDENCIES
    nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  node/include
  target_traj/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package()

FILE(GLOB target_traj_sources
    "target_traj/include/*.h"
    "target_traj/src/*.cpp")

FILE(GLOB uav_sources
    "node/include/*.h"
    "node/include/*.hpp"
    "node/src/*.cpp")


add_executable(ieee_uav_node node/main.cpp  ${uav_sources} ${target_traj_sources})
target_link_libraries(ieee_uav_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES}
  ooqpgensparse ooqpsparse ooqpgondzio ooqpbase blas ma27 gfortran
)
