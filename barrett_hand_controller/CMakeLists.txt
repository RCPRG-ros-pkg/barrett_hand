cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros actionlib rtt_actionlib actionlib_msgs rtt_actionlib_msgs sensor_msgs rtt_sensor_msgs visualization_msgs rtt_visualization_msgs genmsg rtt_roscomm)

add_message_files(DIRECTORY msg FILES BHCmd.msg BHPressureState.msg BHTemp.msg)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs)

ros_generate_rtt_typekit(${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

add_definitions(-DRTT_COMPONENT)
orocos_component(barrett_hand_controller src/CANDev.cpp src/MotorController.cpp src/hand.cpp)
target_link_libraries(barrett_hand_controller ${catkin_LIBRARIES})

#Start of user code user_build
if(${OROCOS_TARGET} STREQUAL "xenomai" )
# message("Found RTNET: ${RTNET_INCLUDE_DIR}")
# include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS} -DXENOMAI")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  orocos_add_compile_flags(barrett_hand_controller ${XENO_CFLAGS})
  orocos_add_link_flags(barrett_hand_controller ${XENO_LDFLAGS})
  TARGET_LINK_LIBRARIES(barrett_hand_controller "-lrtdm")

endif()
#End of user code

orocos_generate_package()

