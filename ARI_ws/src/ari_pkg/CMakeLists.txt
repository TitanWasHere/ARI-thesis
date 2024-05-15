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
  message_generation
)
add_service_files(
  FILES
  msgs.srv
  wavs_msg.srv
  msgPOI.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS scripts/make_wavs.py scripts/speech.py scripts/confirm_mov.py scripts/make_wavs.py scripts/prova.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
