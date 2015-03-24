cmake_minimum_required(VERSION 2.8.3)
project(barrett_ft_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros rtt_roscomm geometry_msgs std_srvs cmake_modules)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

orocos_component(barrett_ft_sensor src/CANDev.cpp src/FtSensor.cpp src/FtSensorComponent.cpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(barrett_ft_sensor ${catkin_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(barrett_ft_sensor ${catkin_LIBRARIES})
endif()

#Start of user code user_build
if(${OROCOS_TARGET} STREQUAL "xenomai" )
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
endif()
#End of user code

orocos_generate_package()

