cmake_minimum_required(VERSION 3.0.2)
project(projectairsim_ros1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  radar_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()
