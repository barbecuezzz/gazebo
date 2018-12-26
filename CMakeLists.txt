cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


include_directories(./program)

add_library(my_exo SHARED my_exo.cc ./program/build_robot.cpp )
target_link_libraries(my_exo
  ${GAZEBO_LIBRARIES}
)
