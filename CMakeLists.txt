cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(my_exo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf 
)


#catkin_package()


#include_directories(${catkin_INCLUDE_DIRS})


#include_directories(include ${catkin_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


include_directories(./program  ${catkin_INCLUDE_DIRS})

add_library(my_exo SHARED my_exo.cc ./program/build_robot.cpp )
target_link_libraries(my_exo
  ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES}
  
)
