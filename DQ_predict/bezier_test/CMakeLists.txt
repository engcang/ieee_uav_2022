cmake_minimum_required(VERSION 3.0.2)
project(bezier_test)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  SYSTEM 
  third_party
  ${catkin_INCLUDE_DIRS} 
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_library( bezier_predict 
    STATIC
    src/bezier_predict.cpp 
)
target_link_libraries( bezier_predict
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ooqpgensparse 
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran 
)  

add_library( predict 
    STATIC
    src/predict.cpp
)
target_link_libraries( predict
    ${catkin_LIBRARIES} 
    bezier_predict
)  


add_executable(predict_test 
  src/predict_test.cpp
)
target_link_libraries(predict_test 
  ${catkin_LIBRARIES}
  bezier_predict
  predict
)
