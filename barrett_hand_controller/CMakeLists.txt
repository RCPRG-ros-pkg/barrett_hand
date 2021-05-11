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
    controller_common
    fabric_logger
)

find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

orocos_component(barrett_hand_controller src/MotorController.cpp src/hand.cpp)

target_link_libraries(barrett_hand_controller ${catkin_LIBRARIES})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS_TARGETS controller_common
)

#catkin_python_setup()

install(TARGETS
  barrett_hand_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)

