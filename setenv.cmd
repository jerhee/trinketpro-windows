@echo off
set PATH=%~dp0\bin\x86\;%~dp0\bin;%PATH%
if "%PROCESSOR_ARCHITECTURE%" == "AMD64" set PATH=%~dp0\bin\amd64;%PATH%

set ARDUINO_DIR=%programfiles%\Arduino
if exist "%ARDUINO_DIR%" goto found

set ARDUINO_DIR=%programfiles(x86)%\Arduino
if exist "%ARDUINO_DIR%" goto found

echo Please ensure that Arduino is installed to Program Files.

:found

set PATH=%ARDUINO_DIR%\hardware\tools\avr\bin;%ARDUINO_DIR%;%PATH%
