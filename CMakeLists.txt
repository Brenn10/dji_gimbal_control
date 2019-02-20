cmake_minimum_required(VERSION 2.8.3)
project(dji_gimbal_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  dji_sdk
  geometry_msgs
  message_generation
  sensor_msgs
  std_srvs
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(DJIOSDK REQUIRED)

add_service_files(DIRECTORY srv
  FILES
  SetBoolean.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES dji_gimbal_control
	CATKIN_DEPENDS dji_sdk geometry_msgs roscpp message_runtime
	DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)
	
add_executable(dji_gimbal_control_node
	src/dji_gimbal_control.cpp
	)

target_link_libraries(dji_gimbal_control_node
	${catkin_LIBRARIES}
	${DJIOSDK_LIBRARIES}
	)