QT -= gui

TEMPLATE = lib
CONFIG += dll
TARGET = pathplanning

DEFINES += PATHPLANNING_LIBRARY

DESTDIR = $$PWD/bin

SOURCES += path_planning.cpp
HEADERS += path_planning.h
