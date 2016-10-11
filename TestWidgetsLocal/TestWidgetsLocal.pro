#-------------------------------------------------
#
# Project created by QtCreator 2016-03-12T14:03:43
#
#-------------------------------------------------

QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TestWidgetsLocal
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    customscene.cpp \
    desenharcampo.cpp \
    robo.cpp \
    ball.cpp \
    RTDB_Module/cicle_iterator.cpp \
    RTDB_Module/externalrtdb.cpp \
    RTDB_Module/filewriter.cpp \
    RTDB_Module/rtdbclient.cpp \
    RTDB_Module/rtdbmainclass.cpp \
    RTDB_Module/sender_bythread.cpp \
    refboxinfo.cpp

HEADERS  += mainwindow.h \
    customscene.h \
    desenharcampo.h \
    robo.h \
    ball.h \
    RTDB_Module/cicle_iterator.h \
    RTDB_Module/externalrtdb.h \
    RTDB_Module/filewriter.h \
    RTDB_Module/paths.h \
    RTDB_Module/rtdbclient.h \
    RTDB_Module/rtdbmainclass.h \
    RTDB_Module/sender_bythread.h \
    RTDB_Module/structs.h \
    refboxinfo.h

FORMS    += mainwindow.ui

RESOURCES += \
    rs.qrc

