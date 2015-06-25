TARGETNAME=i2ctester
TARGETTYPE=program

OPT=-O2

CPPFLAGS= \
    $(CPPFLAGS) \
    -std=c++11 \

CPPSRC=main.cpp

LIBS= \
    ../libarduinocore/libarduinocore.a \
    