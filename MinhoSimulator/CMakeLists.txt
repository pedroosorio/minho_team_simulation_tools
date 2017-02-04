cmake_minimum_required(VERSION 2.8.3)
project(minhosimulator C CXX)

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

set(PROJECT_DIRECTORY ${minhosimulator_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

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
set(MSIM_SRCDIR ${PROJECT_DIRECTORY}/src)
set(msim_MOC_H ${PROJECT_DIRECTORY}/include/action_dialog.h ${PROJECT_DIRECTORY}/include/replay_window.h)
qt5_wrap_cpp(msim_CPP ${msim_MOC_H})
set(msim_SRC ${MSIM_SRCDIR}/main.cpp ${MSIM_SRCDIR}/action_dialog.cpp ${MSIM_SRCDIR}/replay_window.cpp)
add_executable(minhosimulator ${msim_SRC} ${msim_CPP}) 
target_include_directories(minhosimulator PUBLIC
$<BUILD_INTERFACE:${PROJECT_DIRECTORY}/include>)

set_target_properties(minhosimulator PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(minhosimulator PRIVATE cxx_range_for)
target_link_libraries(minhosimulator ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(minhosimulator minho_team_ros_generate_messages_cpp)
######################################################################
