cmake_minimum_required(VERSION 2.8.3)
project(trajectories_batching)

find_package(catkin REQUIRED COMPONENTS
  baxter_mover_utils
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectories_batching
#  CATKIN_DEPENDS baxter_mover_utils moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectories_batching src/trajectories_batching.cpp)
target_link_libraries(trajectories_batching ${catkin_LIBRARIES})