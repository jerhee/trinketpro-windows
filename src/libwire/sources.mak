TARGETNAME=libwire
TARGETTYPE=library

CSRC= \
    twi.c \
    
CPPSRC= \
    Wire.cpp \
    
{c:\program files (x86)\Arduino\hardware\arduino\avr\libraries\Wire\utility}.c{}.o:
    @echo.
	@echo Compiling $<
	$(CC) -c $(CPPFLAGS) $(OPT) $(CINCS) /Arduino/hardware/arduino/avr/libraries/Wire/utility/$(<F)
 
{c:\program files (x86)\Arduino\hardware\arduino\avr\libraries\Wire}.cpp{}.o:
    @echo.
	@echo Compiling $<
	$(CC) -c $(CPPFLAGS) $(OPT) $(CINCS) /Arduino/hardware/arduino/avr/libraries/Wire/$(<F)
