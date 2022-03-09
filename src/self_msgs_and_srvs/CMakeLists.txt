cmake_minimum_required(VERSION 2.8.3)
project(self_msgs_and_srvs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
	roscpp 
	rospy
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  input_point.msg
	output_point.msg
)

add_service_files(
  FILES
  GlbObsRcv.srv
  LearningSampler.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES self_msgs_and_srvs
  CATKIN_DEPENDS message_runtime std_msgs 
#  DEPENDS system_lib
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
)

