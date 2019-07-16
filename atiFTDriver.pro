TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11


SOURCES += \
        main.cpp \
    ft_sensor.cpp \
    netft_rdt_driver.cpp \
    ../dataFile/xml_file.cpp \
    ../util/util_util.cpp \
    ../commun/commun.cpp \
    ../commun/serialcom.cpp \

HEADERS += \
    ft_sensor.h \
    netft_rdt_driver.h \
    ../dataFile/xml_file.h \
    ../util/util.hpp \
    ../util/util_util.h \
    ../commun/serialcom.h \

LIBS += -lboost_system -lboost_thread -lpthread
