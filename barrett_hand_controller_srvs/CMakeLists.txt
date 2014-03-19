cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_controller_srvs)

find_package(catkin REQUIRED COMPONENTS genmsg rtt_roscomm geometry_msgs)

add_message_files(DIRECTORY msg FILES BHPressureInfo.msg BHPressureInfoElement.msg)
add_service_files(DIRECTORY srv FILES Empty.srv BHGetPressureInfo.srv BHMoveHand.srv)
generate_messages(DEPENDENCIES geometry_msgs)

ros_generate_rtt_typekit(${PROJECT_NAME})
ros_generate_rtt_service_proxies(barrett_hand_controller_srvs)

orocos_generate_package()
