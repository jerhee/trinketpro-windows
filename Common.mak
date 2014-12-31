.SUFFIXES:
.SUFFIXES: .c .cpp .S .o .hex .elf .bin .a

# Trinket Pro 3.3V
MCU=atmega328p
F_CPU=12000000L

CFLAGS= \
    -mmcu=$(MCU) \
    -DF_CPU=$(F_CPU) \
    -DARDUINO=150 \
    -DARDUINO_ARCH_AVR \
    -D__PROG_TYPES_COMPAT__ \
    -Wall \
    -ffunction-sections \
    -fdata-sections \
    -fno-exceptions \
    
CPPFLAGS=$(CFLAGS)
ASFLAGS=$(CFLAGS)
LDFLAGS= -mmcu=$(MCU)
OPT= -Os

CINCS= \
    -I/Arduino/hardware/arduino/avr/cores/arduino \
    -I/Arduino/hardware/arduino/avr/variants/eightanaloginputs \
    -I/Arduino/hardware/arduino/avr/libraries/Wire \
    -I/Arduino/hardware/arduino/avr/libraries/Wire/utility \
    -I/Arduino/hardware/arduino/avr/libraries/SPI \

# include the project specific definitions
!include sources.mak
    
PFX = avr
CC = $(PFX)-gcc
CPP = $(PFX)-g++
AR = $(PFX)-ar
OBJCOPY = $(PFX)-objcopy
OBJDUMP = $(PFX)-objdump
SIZE = $(PFX)-size
NM = $(PFX)-nm

OBJ = $(CSRC:.c=.o) $(ASRC:.S=.o) $(CPPSRC:.cpp=.o)

!if "$(TARGETTYPE)" == "program"
TARGET=$(TARGETNAME).hex
CFLAGS=$(CFLAGS) -Werror
CPPFLAGS=$(CPPFLAGS) -Werror
!else if "$(TARGETTYPE)" == "library"
TARGET=$(TARGETNAME).a
!endif

all: $(TARGET) 

$(TARGETNAME).elf: $(OBJ) $(LIBS)
	@echo.
	@echo Linking $@
	$(CC) $(LDFLAGS) $(OPT) -o $@ $(OBJ) $(LIBS)
	@echo Size of executable:
	@$(SIZE) -B -d $(TARGETNAME).elf 

$(TARGETNAME).a: $(OBJ)
    $(AR) r $@ $?
    
.elf.hex:
	@echo.
	@echo Creating HEX
	$(OBJCOPY) -O ihex $< $@

.c.o:
	@echo.
	@echo Compiling $<
	$(CC) -c $(CFLAGS) $(OPT) $(CINCS) $<
    
.cpp.o:
	@echo.
	@echo Compiling $<
	$(CC) -c $(CPPFLAGS) $(OPT) $(CINCS) $<

.S.o:
	$(CC) -c $(ASFLAGS) $(OPT) $< -o $@

upload: upload_serial

.PHONY: upload_serial
upload_serial: $(TARGETNAME).hex
	upload-serial.cmd $(TARGETNAME).hex

# Unplug and replug the board before programming via USB. Beware of USB hubs.
.PHONY: upload_usb
upload_usb: $(TARGETNAME).hex
	avrdude.exe -C /Arduino/hardware/tools/avr/etc/avrdude.conf -c usbtiny -p atmega328p -U flash:w:$(TARGETNAME).hex
    
.PHONY: clean	
clean:
	del /q *.o *.a *.lst *.elf *.bin *.hex _comport.txt
