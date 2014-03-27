cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_controller_srvs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm geometry_msgs sensor_msgs message_generation)

add_message_files(DIRECTORY msg FILES BHCmd.msg BHPressureState.msg BHTemp.msg BHPressureInfo.msg BHPressureInfoElement.msg)
add_service_files(DIRECTORY srv FILES Empty.srv BHGetPressureInfo.srv BHMoveHand.srv)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs message_runtime)
