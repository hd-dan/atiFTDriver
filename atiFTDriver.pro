TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    ft_sensor.cpp \
    netft_rdt_driver.cpp \
    ../dataFile/xml_file.cpp \
    ../commun/commun.cpp \
    ../util/util_util.cpp

HEADERS += \
    ft_sensor.h \
    netft_rdt_driver.h \
    ../dataFile/xml_file.h \
    ../util/util.hpp \
    ../commun/commun.h \
    ../util/util_util.h

LIBS += -lboost_system -lboost_thread
