cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_controller)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rtt_ros
    rtt_roscomm
    barrett_hand_msgs
    geometry_msgs
    sensor_msgs
    cmake_modules
    controller_common)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

orocos_component(barrett_hand_controller src/MotorController.cpp src/hand.cpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(barrett_hand_controller ${catkin_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(barrett_hand_controller ${catkin_LIBRARIES})
endif()

#Start of user code user_build
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
endif()
#End of user code

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS_TARGETS controller_common
)

#catkin_python_setup()
