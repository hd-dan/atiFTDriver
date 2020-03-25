TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11


SOURCES += \
        main.cpp \
    ft_sensor.cpp \
    netft_rdt_driver.cpp \
    ../dataFile/data_file.cpp \
    ../dataFile/xml_file.cpp \
    ../commun/tcpcom.cpp \
    ../commun/serialcom.cpp \

HEADERS += \
    ft_sensor.h \
    netft_rdt_driver.h \
    ../dataFile/data_file.h \
    ../dataFile/xml_file.h \
    ../util/util.hpp \
    ../commun/tcpcom.h \
    ../commun/serialcom.h \

LIBS += -lboost_system -lboost_thread -lpthread
