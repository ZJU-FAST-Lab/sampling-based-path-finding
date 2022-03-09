cmake_minimum_required(VERSION 2.8.3)
project(occ_grid)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES occ_grid
 CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS system_lib
) 

include_directories( 
    include 
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


add_library( occ_grid 
    src/occ_map.cpp 
    src/raycast.cpp
)
target_link_libraries( occ_grid
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)  
