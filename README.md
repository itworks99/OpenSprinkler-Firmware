##OpenSprinklerMicro

---
05/10/2017 Update:
The firmware is now updated to the latest version from main repository
---

OpenSprinklerMicro is a minimalistic irrigation controller running on standart Arduino Uno hardware and shields that is controlled through web interface.

The solenoid valves being controlled through relay board of your choice connected to GPIO pins. Support for LCD, logging, remote stations and sensors is disabled due to the Uno memory size.
 
The contrloller connects to the network as DHCP client using software MAC address **00:69:69:2d:31:00** and hostname **OS-AA**.
 
###Hardware:

The following hardware shields are required:

- Arduino Uno R3
- DS1307 real-time clock shield
- ENC28J60 Ethernet module
- Relay board of your choice
 
![alt tag](OpenSprinklerMicro_bb.png)

You would also need Wire library as it is not included into the firmware source.

###Software:

 The code is shrinked to 30910 bytes (94% of Uno memory). To achieve this, it was compiled with the following options:
 
####GCC:
 
- fno-inline-small-functions
- funsigned-char
- fshort-enums
 
  https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
 
####AVR-specific:
 
- mcall-prologues
 
  https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
 
Please refer to the links above for the detailed description of each option.  
