cmake_minimum_required(VERSION 3.0.2)
project(pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ackermann_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS)