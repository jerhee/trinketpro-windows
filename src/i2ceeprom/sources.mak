TARGETNAME=eeprom
TARGETTYPE=program

OPT=-O2

CSRC=twi.c
CPPSRC=eeprom.cpp Wire.cpp

LIBS= \
    ../libarduinocore/libarduinocore.a \
