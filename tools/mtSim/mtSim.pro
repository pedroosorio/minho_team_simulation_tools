#-------------------------------------------------
#
# Project created by QtCreator 2016-08-12T18:51:59
#
#-------------------------------------------------

QT       += core gui xml
CONFIG += c++11
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets network

TARGET = mtSim
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    renderingcamera.cpp \
    renderview.cpp \
    worldmanager.cpp \
    teleopprocessmanager.cpp \
    fieldbuilderdialog.cpp

HEADERS  += mainwindow.h \
    renderingcamera.h \
    renderview.h \
    worldmanager.h \
    teleopprocessmanager.h \
    fieldbuilderdialog.h

FORMS    += mainwindow.ui \
    fieldbuilderdialog.ui

# Includes for Ignition and Gazebo
INCLUDEPATH += "/usr/local/include/ignition"
INCLUDEPATH += "/usr/include/gazebo-7"
INCLUDEPATH += "/usr/include/ignition/math2"
INCLUDEPATH += "/usr/include/sdformat-4.0"
INCLUDEPATH += "/usr/include"

# Gazebo and Ignition Linkings
LIBS += /usr/lib/x86_64-linux-gnu/libprotobuf.so \
        /usr/lib/x86_64-linux-gnu/libgazebo_transport.so \
        /usr/lib/x86_64-linux-gnu/libgazebo_physics.so \
       # /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so \
       # /usr/lib/x86_64-linux-gnu/libgazebo_gui.so \
        /usr/lib/x86_64-linux-gnu/libgazebo_client.so \
        /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so \
        /usr/lib/x86_64-linux-gnu/libgazebo_math.so \
        /usr/lib/x86_64-linux-gnu/libgazebo_common.so \
        /usr/lib/x86_64-linux-gnu/libgazebo.so \
        /usr/lib/x86_64-linux-gnu/libignition-math2.so \
        /usr/lib/x86_64-linux-gnu/libboost_system.so   \
        /usr/lib/x86_64-linux-gnu/libboost_thread.so

LIBS += `pkg-config --libs ignition-rendering`

# Resource Files
RESOURCES += \
    resources.qrc
