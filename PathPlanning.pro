#-------------------------------------------------
#
# Project created by QtCreator 2026-05-12T10:55:22
#
#-------------------------------------------------

QT -= gui

TEMPLATE = lib
CONFIG += dll
TARGET = pathplanning

DEFINES += PATHPLANNING_LIBRARY

DESTDIR = $$PWD/bin

SOURCES += pathplanning.cpp \
    CCCP_deepseek_cpp_20260126.cpp
HEADERS += pathplanning.h \
    pathplanning_global.h

DISTFILES += \
    PathPlanning.pro.user
