cmake_minimum_required(VERSION 2.8.3)
project(wifi_scan)

find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs tf
                                        message_generation)

add_message_files(FILES AddressRSSI.msg Fingerprint.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy roscpp std_msgs tf message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fingerprint src/fingerprint.cpp src/wifiscan.cpp)
target_link_libraries(fingerprint ${catkin_LIBRARIES} -liw)
add_dependencies(fingerprint wifi_scan_gencpp)

add_executable(fingerprint_rviz src/fingerprint_rviz.cpp)
target_link_libraries(fingerprint_rviz ${catkin_LIBRARIES})
add_dependencies(fingerprint_rviz wifi_scan_gencpp)