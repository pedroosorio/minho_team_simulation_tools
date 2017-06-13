cmake_minimum_required(VERSION 2.8.3)
project(minho_team_simulation_tools C CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation	
	sensor_msgs
	genmsg	
	minho_team_ros	
	cv_bridge
	image_transport
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES minho_team
		CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
		DEPENDS sensor_msgs
	)


set(PROJECT_DIRECTORY ${minho_team_simulation_tools_SOURCE_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#Linking steps for Qt Libraries
########################################
find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
add_definitions (${QT_DEFINITIONS})
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(qt_LIBRARIES Qt5::Widgets Qt5::Core Qt5::Gui)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
########################################

######################################################################
#include_directories(${PROJECT_DIRECTORY}/MinhoSimulator/libs)
#include_directories(${PROJECT_DIRECTORY}/MinhoSimulator/include)
#set(MSIM_SRCDIR ${PROJECT_DIRECTORY}/MinhoSimulator/src)
#set(msim_MOC_H ${PROJECT_DIRECTORY}/MinhoSimulator/include/action_dialog.h ${PROJECT_DIRECTORY}/MinhoSimulator/include/replay_window.h)
#qt5_wrap_cpp(msim_CPP ${msim_MOC_H})
#set(msim_SRC ${MSIM_SRCDIR}/main.cpp ${MSIM_SRCDIR}/action_dialog.cpp ${MSIM_SRCDIR}/replay_window.cpp
#    ${MSIM_SRCDIR}/replay.cpp)
#add_executable(minhosimulator ${msim_SRC} ${msim_CPP}) 
#target_include_directories(minhosimulator PUBLIC
#$<BUILD_INTERFACE:${PROJECT_DIRECTORY}/include>)

#set_target_properties(minhosimulator PROPERTIES COMPILE_FLAGS "-fPIC")
#target_compile_features(minhosimulator PRIVATE cxx_range_for)
#target_link_libraries(minhosimulator ${catkin_LIBRARIES} ${qt_LIBRARIES})
#add_dependencies(minhosimulator minho_team_ros_generate_messages_cpp)
######################################################################

##################
## Build Plugin ##
##################
set(HARDWARE_DIR ${PROJECT_DIRECTORY}/robot_plugin)
include_directories(${HARDWARE_DIR}/include)
add_library(minho_robot SHARED ${HARDWARE_DIR}/src/minho_robot.cc)
set_target_properties( minho_robot
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/plugins
)
target_link_libraries(minho_robot ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(minho_robot minho_team_ros_generate_messages_cpp)

set(OTHER_DIR ${PROJECT_DIRECTORY}/other_plugin)
include_directories(${OTHER_DIR}/include)
add_library(other_robot SHARED ${OTHER_DIR}/src/other_robot.cc)
set_target_properties( other_robot
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/plugins
)
target_link_libraries(other_robot ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(other_robot minho_team_ros_generate_messages_cpp)

set(BSR_DIR ${PROJECT_DIRECTORY}/bs_robot_plugin)
include_directories(${BSR_DIR}/include)
add_library(bs_rplugin SHARED ${BSR_DIR}/src/bs_rplugin.cc)
set_target_properties( bs_rplugin
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/plugins
)
target_link_libraries(bs_rplugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(bs_rplugin minho_team_ros_generate_messages_cpp)

set(WORLD_DIR ${PROJECT_DIRECTORY}/world_plugin)
include_directories(${WORLD_DIR}/include)
add_library(minho_world SHARED ${WORLD_DIR}/src/minho_world.cc)
set_target_properties( minho_world
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_DIRECTORY}/plugins
)
target_link_libraries(minho_world ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(minho_world minho_team_ros_generate_messages_cpp)
