#-------------------------------------------------
#
# Project created by QtCreator 2016-07-01T10:26:57
#
#-------------------------------------------------

QT       += core

QT       += gui
QT       += positioning

TARGET   = ub-anc-planner
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

#
# CPLEX Optimization Library
#
include(cplex.pri)

#
# Concorde TSP Library
#
include(cc_lkh.pri)

HEADERS += \
    Waypoint.h \
    UBConfig.h \
    UBPlanner.h \
    LKHPlanner.h \

SOURCES += \
    main.cpp \
    Waypoint.cc \
    UBPlanner.cpp \
    LKHPlanner.cpp \
    linkern.c \

INCLUDEPATH += \
    mavlink/include/mavlink/v2.0 \
    mavlink/include/mavlink/v2.0/ardupilotmega \
