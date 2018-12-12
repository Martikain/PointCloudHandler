QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

DEFINES += QT_DEPRECATED_WARNINGS
CONFIG += link_pkgconfig
PKGCONFIG += eigen3

SOURCES += \
        main.cpp \
    pcdparse.cpp \
    util.cpp

HEADERS += \
    pcdparse.h \
    util.h


INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/vtk-6.2
INCLUDEPATH += /usr/include/boost

LIBS += -lpcl_visualization -lpcl_common -lpcl_io -lboost_system
