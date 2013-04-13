This archive contains the source code, make files, etc for "video-peggy-twi"
Copyright 2008 by Jay Clegg.  All rights reserved.  Released under 
GPL license.

//////////////////////////////////////////////////////////////////////////////
I am releasing this code and the information for how it works 'as-is' and 
make *NO* guarantees that it will not cause damage to your peggy2, your
Arduino, your computer, your immediate surroundings, your health, or your 
sanity.
//////////////////////////////////////////////////////////////////////////////

More information about this project, here
http://www.planetclegg.com/projects/Twi2Peggy.html

In brief: The C firmware code is intended to be installed on an
unmodified Peggy 2.0.   It sets the Peggy up as a TWI/I2C slave device,
and requires a TWI Master to drive the display.  Makefile is included.  
You will need avr-gcc toolchaing (WinAVR or AvrMacPack toolchains work well)
to build this project, and you will need a AVR programmer that is 
compatible with AVRDUDE and that supports a 6-pin ISP header to load the code.
If you dont have a programmer but have a FTDI cable, you can use that assuming
that the AVR has the Arduino bootloader installed.  (See the section at the 
bottom of this file if you want to try to get the firmware working using the
Arduino IDE and FTDI cable rather than AVR-GCC.)

You should modify the makefile to match the AVR programmer you intend to use.
The values are well marked at the top of the makefile.  You should then be
able to do a 'make' followed by a 'make install' to load the code on the chip.

Also included: Two Arduino sketches (.pde files) one that just sends an
animated test pattern from the Arduino to the peggy, and another that
translates Serial to TWI (so that you can drive the display from a PC).

Also included:  A python script that will send a test pattern to a serial 
port, so that you can see if everything is working. You'll need PySerial 
installed, and you'll need to modify the script to match whatever serial 
port you are using.  You would use this to send the output to the
Arduino which then forwards the data to the Peggy.  Edit the script
and set the serial port filename before using it.

README.txt                  Er, this file.
COPYING.txt                  License.  Read it!
Makefile                    Make file for Peggy code
main.c                      Peggy code
video-peggy-twi.bak.hex     Compiled Peggy code
Serial2TwiPeggy.pde         Arduino code to convert serial from PC to TWI
TwiSendTestPattern.pde      Arduino code to send a test pattern
peggytest.py                Python code to send a test pattern, use
                               with Arduino and Serial2TwiPeggy.pde 
                               


=== Building the Video Peggy firmware with Arduino IDE rather than AVR-GCC: ===

If you don't have an AVR toolchain but have the Arduino IDE, you *may* be able 
to get the source code to run as an Arduino sketch and uploadit with a FTDI 
cable by making some minor modifications.

I've deliberately not included a ported version of the Video Peggy firmware 
so as not to cause confusion about what code gets installed on the Peggy 
versus what code gets installed on the Arduino.  But the steps needed to
do a port are as follows:

	1) copy main.c code into the Arduino IDE as a new sketch
	2) comment out the #include<> statements at the top of the file
	3) change the signature for "int main(void)" to "void setup()"
	4) create an empty "void loop() {}" method to keep the compiler happy...
	5) make sure the FPS variable in the source is 90 or possibly less.
	6) Use the FTDI cable to upload the compiled code to the Peggy.
	7) Try not to get the resulting PDE mixed up with those that are
	    intended for the Arduino.  Two I2C masters == potential anti-goodness.
	    
I prefer the avr-gcc approach, as this generates more optimal code by default.  
But I understand that some Peggy users may lack a full toolchain, so
I've included these instructions to help, athough I have not thoroughly tested 
an Arduino IDE based build of the firmware.  

//////////////////////////////////////////////////////////////////////////

This code is derived from above. Interrupt and main loop swapped around,
to make the slave I2C run off the interrupt, and the video display run as main task.

Of course, all run within freeRTOS.



