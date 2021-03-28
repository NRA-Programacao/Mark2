cmake_minimum_required(VERSION 2.8.3)
project(controle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(controle src/controle.cpp)
target_link_libraries(controle ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")