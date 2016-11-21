##OpenSprinklerMicro

OpenSprinklerMicro is a minimalistic version of the OpenSprinkler irrigation controller that can run on standart Arduino Uno hardware and shields.

####Hardware:

 - Arduino Uno R3
 - DS1307 real-time clock shield
 - ENC28J60 Ethernet module
 - Relay board and (optionally) a shift register
 
The support of shift register could be reenabled through defines.h while the current code is written to support relay board connected directly to the pins 0-7.

The RTC module (DS1307) seems to have issues causing I2C bus to freeze - this problem (and solution to it) was described by Paul O'Dowd in his blog post:

http://www.paulodowd.com/2015/04/ds1307-woes-i2c-freezes-and-locks.html

so the code now uses WsWire instead of standart Wire library. DS1307 is connected directly to A2-A5 pins.

####Network:

By default it connects to the network as DHCP client using software MAC address 00:69:69:2d:31:00 and hostname OS-AA.

####Software:

Support of the rain sensor could be reenabled through define.h. The other features, such as:

 - LCD and buttons
 - Logging and support of SD card
 - RF station support
 
 were removed/disabled due to the Uno storage/memory constraints. The code is shrinked to 31200 bytes (95% of Uno memory). To achieve this, it was compiled with the following options:
 
 -mcall-prologues
 -fno-inline-small-functions
 -funsigned-char
 -fshort-enums
 
 Please refer to the following link for the detailed description of each option:
 
 https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
 https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
 
 I beleive that support of the flow sensor could be reenabled in the future with the further optimization of the source code.
  