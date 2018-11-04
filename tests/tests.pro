QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
        main.cpp \
    conversiontests.cpp \
    ../src/pcdparse.cpp \
    ../src/util.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    conversiontests.h \
    ../src/pcdparse.h \
    ../src/util.h \
    defines.h

INCLUDEPATH += /usr/include/gtest
INCLUDEPATH += ../src
INCLUDEPATH += $$(HOME)/eigen3
INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/vtk-6.2
INCLUDEPATH += /usr/include/boost

LIBS += -lpcl_visualization -lpcl_common -lpcl_io -lboost_system
LIBS += -lgtest

