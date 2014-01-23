cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros actionlib rtt_actionlib actionlib_msgs rtt_actionlib_msgs sensor_msgs rtt_sensor_msgs visualization_msgs rtt_visualization_msgs genmsg rtt_roscomm)

add_action_files(DIRECTORY action FILES BHMove.action)
add_message_files(DIRECTORY msg FILES BHFingerVel.msg BHPressureState.msg BHTemp.msg)
#add_message_files(DIRECTORY msg FILES BHFingerVel.msg BHPressureInfo.msg BHPressureState.msg BHPressureInfoElement.msg BHTemp.msg)
#add_service_files(DIRECTORY srv FILES BHCalibrateTactileSensors.srv BHGetPressureInfo.srv)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs)

ros_generate_rtt_typekit(${PROJECT_NAME})

catkin_package()

#ros_generate_rtt_service_proxies(${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

add_definitions(-DRTT_COMPONENT)
orocos_component(${PROJECT_NAME} src/CANDev.cpp src/MotorController.cpp src/hand.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package()

install(FILES 
  single_hand.launch 
  single_hand.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

