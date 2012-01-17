cmake_minimum_required(VERSION 2.8)
project(darc_ros)

set(CMAKE_PREFIX_PATH /opt/ros/fuerte)

find_package(catkin)
find_package(darc)
find_package(Boost COMPONENTS system thread)
find_package(ROS REQUIRED COMPONENTS cpp_common rostime roscpp_traits roscpp_serialization roscpp rosconsole darc)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ROS_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall -g")
#set(CMAKE_CXX_FLAGS "-Os")

add_executable(darc_ros src/darc_ros.cpp src/node_darc_ros.cpp)
target_link_libraries(darc_ros ${Boost_LIBRARIES} ${ROS_LIBRARIES})

catkin_project(darc_ros
  VERSION 0.0.1
#  INCLUDE_DIRS include
  )
