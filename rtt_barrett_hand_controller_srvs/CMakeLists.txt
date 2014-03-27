cmake_minimum_required(VERSION 2.8.3)
project(rtt_barrett_hand_controller_srvs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(barrett_hand_controller_srvs)
ros_generate_rtt_service_proxies(barrett_hand_controller_srvs)

orocos_generate_package(
  DEPENDS barrett_hand_controller_srvs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_rosgraph_msgs
 rtt_geometry_msgs
 rtt_sensor_msgs

)
