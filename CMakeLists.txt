cmake_minimum_required(VERSION 2.8.3)
project(wifi_scan)

find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs 
                                        message_generation)

add_message_files(
  FILES
  rssi.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs message_runtime
)
