cmake_minimum_required(VERSION 3.0.2)
project(ari_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #pal_navigation_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
