TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    tsp.cpp \
    cvrp.cpp

HEADERS += \
    tsp.hpp \
    cvrp.hpp \
    types.hpp
