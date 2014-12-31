TARGETNAME=libarduinospi
TARGETTYPE=library

CPPSRC= \
    SPI.cpp \
    
{c:\program files (x86)\Arduino\hardware\arduino\avr\libraries\SPI}.cpp{}.o:
    @echo.
	@echo Compiling $<
	$(CC) -c $(CPPFLAGS) $(OPT) $(CINCS) /Arduino/hardware/arduino/avr/libraries/SPI/$(<F)
