##Windows Makefiles for Adafruit Trinket Pro 3.3V

This project contains Makefiles and project skeletons for 
[Adafruit Trinket Pro 3.3V](http://www.adafruit.com/product/2010). The Makefiles
are written for Windows's native build tool [nmake](http://msdn.microsoft.com/en-us/library/dd9y37ha.aspx) and does not require installation of cygwin. This project has
an external dependency on Arduino, which is used for avr-gcc, avrdude, and
Arduino libraries. Code is compiled as straight C++, so any code that works
in this project will work in the Arduino IDE, though not necessarily the other
way around since Arduino lets you write malformed C++ code. 

###Build and upload blinky from the command line

 1. Download and install the Arduino Windows installer from [http://www.arduino.cc/en/Main/Software](http://www.arduino.cc/en/Main/Software).
    Accept the default install location (C:\Program Files (x86)\Arduino).
    This is important, as the scripts expect this location.

 1. Clone this repository
    
        git clone https://github.com/jerhee/trinketpro-windows.git  
    
 1. CD to repository and put tools on your path

        cd trinketpro-windows
        setenv.cmd
 
 1. Build
        
        cd src
        nmake

 1. Plug in a [USB to serial cable](https://www.sparkfun.com/products/12977)
    to the Trinket
 
 1. Upload blinky

        cd blink
        nmake upload
    
###Uploading over USB (without FTDI cable)

 1. Install [Trinket Pro USB drivers](https://learn.adafruit.com/introducing-pro-trinket/downloads).
    Unzip the file, then right click on USBTiny.inf and click Install.
 1. Plug a micro-USB cable into the trinket
 1. nmake upload_usb

I have found that USB upload is flaky and only works ~10% of the time,
so I recommend using serial if possible. You can increase
the chances of success by unplugging and replugging the USB cable
each time you program it, and by not using a USB hub. 