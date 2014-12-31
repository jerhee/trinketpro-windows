@echo off
setlocal enableextensions disabledelayedexpansion

if "%1"=="" echo Please specify an input file: program-serial.cmd input_file.hex && exit /b 1

:: get the cached COM port or prompt user
if exist _comport.txt (
    set /p COMPORT=<_comport.txt
) else (
    call :prompt
)

echo Programming board using COM%COMPORT%
avrdude.exe -C /Arduino/hardware/tools/avr/etc/avrdude.conf -c arduino -p atmega328p -U flash:w:%1 -P COM%COMPORT%
set ret=%errorlevel%
if not "%ret%"=="0" del _comport.txt
exit /b %ret%

:prompt
    echo Please select a COM port:
    echo.
    "%ARDUINO_DIR%\hardware\tools\listComPorts.exe"
    echo.
    set /p COMPORT="Enter COM number (e.g. type '3' to select COM3): "
    echo %comport% > _comport.txt
    exit /b 0
