cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_tactile)


find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros rtt_roscomm barrett_hand_controller_msgs geometry_msgs sensor_msgs cmake_modules can_driver)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
endif()

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(barrett_hand_tactile src/TactileInterface.cpp src/Tactile.cpp src/hand_tactile.cpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(barrett_hand_tactile ${catkin_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(barrett_hand_tactile ${catkin_LIBRARIES})
endif()

#Start of user code user_build
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
endif()
#End of user code

orocos_generate_package()

