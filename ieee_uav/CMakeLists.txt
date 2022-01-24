cmake_minimum_required(VERSION 2.8.12)
project(ieee_uav)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  pcl_conversions
  pcl_ros
  yolo_ros_simple
  mavros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package()

add_executable(ieee_uav_node src/main.cpp)
target_link_libraries(ieee_uav_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES}
)