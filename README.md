# Description

AVR ATmega port of freeRTOS
A port of freeRTOS which can be flexibly flexibly configured use almost any available Timer on AVR ATmega devices and will operate with almost any classic Arduino device from Arduino, SeeedStudio, Sparkfun, Freetronics or Pololu.

The [Goldilocks Analogue story of adding a dual channel DAC to a 1284p classic Arduino board is told here](http://feilipu.me/?s=Goldilocks+Analogue).
The [Goldilocks Analogue - Kickstarter Campaign](https://www.kickstarter.com/projects/feilipu/goldilocks-analogue-classic-arduino-audio-superpow/) was successfully funded. 
It was also a Hackaday Prize 2015 Quarter Finalist.

![Goldilocks Analogue](https://a.fsdn.com/con/app/proj/avrfreertos/screenshots/Title%20Image.JPG "Goldilocks Analogue")

The first [Goldilocks 1284p story is here](http://feilipu.me/2013/03/08/goldilocks-1284p-arduino-uno-clone/).
Ths idea was launched as a [Pozible campaign in 2013](http://www.pozible.com/goldilocks/).
It was successfully funded and the [Goldilocks](http://freetronics.com/goldilocks/) was developed and supported by Freetronics. 

This repo has been hosted on [Sourceforge avrfreertos](https://sourceforge.net/projects/avrfreertos/) since 2011, and has been [downloaded over 25,000 times](https://sourceforge.net/projects/avrfreertos/files/stats/timeline?dates=2011-09-22+to+2018-8-31).
Going forward Sourceforge will be updated less often and only with major releases.

[Follow @_feilipu](https://twitter.com/_feilipu) on Twitter.

# Getting Started

To get started, follow the instructions on [freeRTOS & libraries for AVR ATmega](http://feilipu.me/freertos-and-libraries-for-avr-atmega).
Then do some further reading on the [freeRTOS Quick Start Guide](http://www.freertos.org/FreeRTOS-quick-start-guide.html).

If this repository is too complicated to get started, a [minimum AVR freeRTOS configured using the Watchdog Timer and Heap 3](https://github.com/feilipu/miniAVRfreeRTOS) is also also available.
This option is without libraries, and is configured to get started with simple applications.

Also, for the Arduino platform, there is an [Arduino freeRTOS Library](https://github.com/feilipu/Arduino_FreeRTOS_Library)
available in the Arduino IDE Library manager, or by directly downloading the ZIP file and importing it into your Arduino IDE.

# Features
- freeRTOS 10.1.1 implemented for selected AVR ATmega devices
- Arduino Uno, Pro, Mini, Nano, & LilyPad with ATmega328p supported
- Arduino Mega (Seeed ADK, Freetronics EtherMega) with ATmega2560 supported
- Goldilocks (Analogue) & Pololu Orangutan SVP with ATmega1284p supported
- Integrated Timer2 for real time system_time operation with 32.768kHz Crystal
- [Multiple DAC Support](https://github.com/feilipu/avrfreertos/blob/master/freeRTOS9xx/include/DAC.h): Goldilocks Analogue MCP4822 and the DAC8564 & WM8731.
- Master SPI Mode on ATmega1284p USART1 for MCP4822 DAC
- [ChaN's SD Card FAT FS library](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_fatf) (Update July 4, 2017: v0.13p1)
- [Standard C90 Library](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_time) from avr-libc upstream for standard and esoteric time calculations
- [IP protocols](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_inet), DHCP, NTP, PING, HTTP web server for WIZNET W5x00
- [W5500 Driver](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_iinchip), same BSD Socket API as W5100
- [W5200 v1.3 Driver](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_iinchip), same BSD Socket API as W5100
- [W5100 v1.6.1 Driver](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_iinchip) with BSD Socket API
- [USB Host support through MAX3421E](https://github.com/feilipu/avrfreertos/tree/master/microbridge) for Seeed ADK (and Shields).
- [EEFS (NASA Flash File System v2.0)](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_eefs) for SPI EEPROM, FRAM, SRAM
- [uIP (working) and uIPv6 (untested)](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib-uIP) on Wiznet (IINChip) W5x00 MACRAW
- uIP support for BlackWidow WiFi (TESTING ONLY)
- [Library for 2560 XRAM](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_ext_ram) using Rugged Circuits QuadRAM (512kB) & MegaRAM (128kB)
- [XRAMFS (16x Arduino Clients sharing 512kByte Supervisor SDRAM) Supervisor](https://github.com/feilipu/avrfreertos/tree/master/ramfs_supervisor) and [Client](https://github.com/feilipu/avrfreertos/tree/master/ramfs_load_gen) for [ArduSat](http://feilipu.me/?s=ArduSat).
- [Abstract Serial to support multiple USART (1284p, 2560)](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_io) with [fast ring buffers](https://github.com/feilipu/avrfreertos/blob/master/freeRTOS9xx/include/ringBuffer.h)
- [XBee API Mode](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_xbee) support.
- [FTDI FT800 EVE](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_ft800) support for the Gameduino2 (1284p, 328p) & 4D Systems ADAM
- [HD44780 for Freetronics LCD (and other) Shields](https://github.com/feilipu/avrfreertos/tree/master/freeRTOS9xx/lib_hd44780) (based on ChaN)
- [Nokia 6100 LCD](https://github.com/feilipu/avrfreertos/tree/master/LCD6100_Driver) support.
- [stk500v2 (wiring) bootloader (with boot monitor) for Arduino Mega2560 & Goldilocks 1284p](https://github.com/feilipu/avrfreertos/tree/master/AVRstk500v2_bootloader).

The freeRTOS scheduler can be driven by any of the AVR ATmega timers, down to one millisecond (or less) time slices, depending on the application requirements.
By using Timer2 for scheduling timing can be linked with a RTC capability with up to millisecond accuracy.
By using the Watchdog Timer for scheduling no externally useful system resources are consumed, but time slicing is limited to 15 millisecond increments.
Generally, I use the freeRTOS scheduler for loose timing (15ms) and hardware timers for tight (microsecond) timing, but each application is different.

There are [multiple options for configuring the heap in freeRTOS](http://www.freertos.org/a00111.html). Please read the freeRTOS descriptions for the appropriate solution relevant to the application.
Heap versions 1, 2, 3, and 4 have been implemented for this repository.

An integrated RTC providing [Standard C90 system_time](http://www.nongnu.org/avr-libc/user-manual/group__avr__time.html) using the 32.768kHz crystal attached to Timer2 on 1284p Goldilocks or other RTC equipped boards.
Standard C90 system_time is also available as an approximate RTC off other timers using 16MHz and other crystals.

# Further Reading

The canonical source for information is the [freeRTOS Web Site](http://www.freertos.org/).
Within this site, the [Getting Started](http://www.freertos.org/FreeRTOS-quick-start-guide.html) page is very useful.
It is worth having a view from another user, and [manicbug](https://maniacbug.wordpress.com/2012/01/31/freertos/) has some interesting examples.
This AVRfreeRTOS Repository has plenty of examples, ranging from [Blink](https://github.com/feilipu/avrfreertos/blob/master/MegaBlink/main.c) through to a [Synthesiser](https://github.com/feilipu/avrfreertos/tree/master/GA_Synth) for the Goldilocks Analogue.
