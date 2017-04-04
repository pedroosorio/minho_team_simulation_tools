cmake_minimum_required(VERSION 2.8.3)
project(minho_team_station C CXX)

set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

set(PROJECT_DIRECTORY ${minho_team_station_SOURCE_DIR})
include_directories($ENV{HOME}/Common)

######################################################################
#Linking steps for ROS and Gazebo Libraries
########################################
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
find_package(ignition-math2 REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
message(STATUS ${IGNITION-MATH_INCLUDE_DIRS})
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
include_directories(/usr/include/sdformat-4.0)


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
include_directories(${PROJECT_DIRECTORY}/libs)
set(MBS_SRCDIR ${PROJECT_DIRECTORY}/src)
set(mbs_MOC_H ${PROJECT_DIRECTORY}/include/gzwidget.h ${PROJECT_DIRECTORY}/include/mainwindow.h ${PROJECT_DIRECTORY}/include/robotwidget.h)
qt5_wrap_cpp(mbs_CPP ${mbs_MOC_H})
set(mbs_SRC ${MBS_SRCDIR}/main.cpp ${MBS_SRCDIR}/gzwidget.cpp ${MBS_SRCDIR}/mainwindow.cpp ${MBS_SRCDIR}/robotwidget.cpp ${MBS_SRCDIR}/multicastpp.cpp
    ${MBS_SRCDIR}/thpool.c)
add_executable(basestation ${mbs_SRC} ${mbs_CPP}) 
target_include_directories(basestation PUBLIC
$<BUILD_INTERFACE:${PROJECT_DIRECTORY}/include>)

set_target_properties(basestation PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(basestation PRIVATE cxx_range_for)
target_link_libraries(basestation ${IGNITION-MATH_LIBRARIES} ${catkin_LIBRARIES} ${qt_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(basestation minho_team_ros_generate_messages_cpp)
######################################################################

###################
## Build Plugins ##
###################
######################################################################
set(BSPLUGIN_DIR ${PROJECT_DIRECTORY}/bs_plugin)
include_directories(${BSPLUGIN_DIR}/include)
add_library(bs_plugin SHARED ${BSPLUGIN_DIR}/src/bs_plugin.cc)
set_target_properties( bs_plugin
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY $ENV{HOME}/catkin_ws/src/minho_team_simulation_tools/plugins
)
target_link_libraries(bs_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(bs_plugin minho_team_ros_generate_messages_cpp)
######################################################################
