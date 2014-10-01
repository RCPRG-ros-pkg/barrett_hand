cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros rtt_roscomm barrett_hand_controller_srvs geometry_msgs sensor_msgs cmake_modules)

find_package(Xenomai REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

orocos_component(barrett_hand_controller src/CANDev.cpp src/MotorController.cpp src/Tactile.cpp src/hand.cpp)
target_link_libraries(barrett_hand_controller ${catkin_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})

#Start of user code user_build
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
endif()
#End of user code

orocos_generate_package()

