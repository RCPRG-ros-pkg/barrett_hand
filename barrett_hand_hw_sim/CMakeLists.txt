# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_hw_sim)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    cmake_modules
    kdl_parser
    lwr_msgs
    barrett_hand_common
    barrett_hand_controller_msgs
    velma_core_ve_re_lwr_msgs
    eigen_conversions
    common_behavior
    common_interfaces
    test_ec_msgs_gen
    shm_comm
    ec_can_queue
    barrett_hand_controller
)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
  )

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

#set(CMAKE_BUILD_TYPE Debug)


## Default component
orocos_library(barrett_hand_hw_sim
    src/barrett_hand_hw_can.cpp
)

#set_target_properties(velma_sim PROPERTIES COMPILE_FLAGS -DRTT_COMPONENT)
target_link_libraries(barrett_hand_hw_sim
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

orocos_generate_package(
  INCLUDE_DIRS include
)

# Install targets
#install(TARGETS
#  lwr_controller
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  )
