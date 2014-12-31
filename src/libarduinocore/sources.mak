TARGETNAME=libarduinocore
TARGETTYPE=library

CSRC= \
	hooks.c \
	WInterrupts.c \
	wiring.c \
	wiring_analog.c \
	wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c \
	
CPPSRC= \
	abi.cpp \
	CDC.cpp \
	HardwareSerial.cpp \
	HardwareSerial0.cpp \
	HardwareSerial1.cpp \
	HardwareSerial2.cpp \
	HardwareSerial3.cpp \
	HID.cpp \
	IPAddress.cpp \
	main.cpp \
	new.cpp \
	Print.cpp \
	Stream.cpp \
	Tone.cpp \
	USBCore.cpp \
	WMath.cpp \
	WString.cpp \
	
{c:\program files (x86)\Arduino\hardware\arduino\avr\cores\arduino}.c{}.o:
	@echo.
	@echo Compiling $<
	$(CC) -c $(CPPFLAGS) $(OPT) $(CINCS) /Arduino/hardware/arduino/avr/cores/arduino/$(<F)
 
{c:\program files (x86)\Arduino\hardware\arduino\avr\cores\arduino}.cpp{}.o:
	@echo.
	@echo Compiling $<
	$(CC) -c $(CPPFLAGS) $(OPT) $(CINCS) /Arduino/hardware/arduino/avr/cores/arduino/$(<F)
