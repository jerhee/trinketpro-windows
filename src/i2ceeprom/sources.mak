TARGETNAME=eeprom
TARGETTYPE=program

OPT=-O2

CPPSRC=eeprom.cpp

LIBS= \
    ../libarduinocore/libarduinocore.a \
    ../libarduinowire/libarduinowire.a \