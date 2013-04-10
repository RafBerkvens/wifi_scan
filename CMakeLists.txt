cmake_minimum_required(VERSION 2.8.3)
project(wifi_scan)

find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs 
                                        message_generation)

add_message_files(FILES RSSI.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy roscpp std_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fingerprint src/fingerprint.cpp)
target_link_libraries(fingerprint ${catkin_LIBRARIES} -liw)
add_dependencies(fingerprint wifi_scan_gencpp)
