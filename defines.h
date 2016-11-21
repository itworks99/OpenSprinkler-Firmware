/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * OpenSprinkler macro defines and hardware pin assignments
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef _DEFINES_H
#define _DEFINES_H

#define OSMICRO			// OpenSprinklerMicro main define to disable parts of existing code
#define NORF			// Disable support of all special stations (RF, remote etc.) to fit code into Uno memory
#define NOSD			// Disable support of logging functions to fit code into Uno memory
#define NOSHIFT			// Use pins instead of shift register
//#define NORAINSENSOR		// Disable rainsensor
#define NOFLOWSENSOR		// Disable flowsensor to fit code into Uno memory

//#define SERIAL_DEBUG		// Enable debug output via serial

#define TMP_BUFFER_SIZE      96   // scratch buffer size

/** Firmware version, hardware version, and maximal values */
#define OS_FW_VERSION  217  // Firmware version: 217 means 2.1.7
                            // if this number is different from the one stored in non-volatile memory
                            // a device reset will be automatically triggered

#define OS_FW_MINOR      0  // Firmware minor version

/** Hardware version base numbers */
#define OS_HW_VERSION_BASE   0x00
#define HW_TYPE_UNIV         0x15   // DC powered, universal driver

#ifndef NOSD
/** File names */
#define WEATHER_OPTS_FILENAME "wtopts.txt"    // weather options file
#define STATION_ATTR_FILENAME "stns.dat"      // station attributes data file
#endif

#define STATION_SPECIAL_DATA_SIZE  (TMP_BUFFER_SIZE - 8)

#define FLOWCOUNT_RT_WINDOW   30    // flow count window (for computing real-time flow rate), 30 seconds

/** Station type macro defines */
#define STN_TYPE_STANDARD    0x00
#define STN_TYPE_RF          0x01
#define STN_TYPE_REMOTE      0x02
#define STN_TYPE_GPIO        0x03	// Support for raw connection of station to GPIO pin
#define STN_TYPE_HTTP        0x04	// Support for HTTP Get connection
#define STN_TYPE_OTHER       0xFF

/** Sensor type macro defines */
#define SENSOR_TYPE_NONE    0x00
#define SENSOR_TYPE_RAIN    0x01  // rain sensor
#define SENSOR_TYPE_FLOW    0x02  // flow sensor
#define SENSOR_TYPE_PSWITCH 0xF0  // program switch
#define SENSOR_TYPE_OTHER   0xFF

/** Non-volatile memory (NVM) defines */

/** 1KB NVM (ATmega328) data structure:
 * |         |     |  ---STRING PARAMETERS---      |           |   ----STATION ATTRIBUTES-----      |          |
 * | PROGRAM | CON | PWD | LOC | JURL | WURL | KEY | STN_NAMES | MAS | IGR | MAS2 | DIS | SEQ | SPE | OPTIONS  |
 * |  (493)  |(12) |(36) |(48) | (40) | (40) |(24) |   (128)   | (1) | (1) |  (1) | (1) | (1) | (1) |  (58)    |
 * |         |     |     |     |      |      |     |           |     |     |      |     |     |     |          |
 * 0        493   505   541   589    629    669   693        821    822   823    824   825   826   827        885
 */

#define MAX_EXT_BOARDS    0  // maximum number of exp. boards (each expands 8 stations)
#define MAX_NUM_STATIONS  ((1+MAX_EXT_BOARDS)*8)  // maximum number of stations

#define NVM_SIZE            1024  // For AVR, nvm data is stored in EEPROM, ATmega328 has 1K EEPROM
#define STATION_NAME_SIZE   16    // maximum number of characters in each station name

#define MAX_PROGRAMDATA     493   // program data
#define MAX_NVCONDATA       12     // non-volatile controller data
#define MAX_USER_PASSWORD   36    // user password
#define MAX_LOCATION        48    // location string
#define MAX_JAVASCRIPTURL   40    // javascript url
#define MAX_WEATHERURL      40    // weather script url
#define MAX_WEATHER_KEY     24    // weather api key,

/** NVM data addresses */
#define ADDR_NVM_PROGRAMS      (0)   // program starting address
#define ADDR_NVM_NVCONDATA     (ADDR_NVM_PROGRAMS+MAX_PROGRAMDATA)
#define ADDR_NVM_PASSWORD      (ADDR_NVM_NVCONDATA+MAX_NVCONDATA)
#define ADDR_NVM_LOCATION      (ADDR_NVM_PASSWORD+MAX_USER_PASSWORD)
#define ADDR_NVM_JAVASCRIPTURL (ADDR_NVM_LOCATION+MAX_LOCATION)
#define ADDR_NVM_WEATHERURL    (ADDR_NVM_JAVASCRIPTURL+MAX_JAVASCRIPTURL)
#define ADDR_NVM_WEATHER_KEY   (ADDR_NVM_WEATHERURL+MAX_WEATHERURL)
#define ADDR_NVM_STN_NAMES     (ADDR_NVM_WEATHER_KEY+MAX_WEATHER_KEY)
#define ADDR_NVM_MAS_OP        (ADDR_NVM_STN_NAMES+MAX_NUM_STATIONS*STATION_NAME_SIZE) // master op bits
#define ADDR_NVM_IGNRAIN       (ADDR_NVM_MAS_OP+(MAX_EXT_BOARDS+1))  // ignore rain bits
#define ADDR_NVM_MAS_OP_2      (ADDR_NVM_IGNRAIN+(MAX_EXT_BOARDS+1)) // master2 op bits
#define ADDR_NVM_STNDISABLE    (ADDR_NVM_MAS_OP_2+(MAX_EXT_BOARDS+1))// station disable bits
#define ADDR_NVM_STNSEQ        (ADDR_NVM_STNDISABLE+(MAX_EXT_BOARDS+1))// station sequential bits
#define ADDR_NVM_STNSPE        (ADDR_NVM_STNSEQ+(MAX_EXT_BOARDS+1)) // station special bits (i.e. non-standard stations)
#define ADDR_NVM_OPTIONS       (ADDR_NVM_STNSPE+(MAX_EXT_BOARDS+1))  // options

/** Default password, location string, weather key, script urls */
#define DEFAULT_PASSWORD          "a6d82bced638de3def1e9bbb4983225c"  // md5 of 'opendoor'
#define DEFAULT_LOCATION  	  "Sydney,Australia"
#define DEFAULT_WEATHER_KEY       ""
#define DEFAULT_JAVASCRIPT_URL    "https://ui.opensprinkler.com/js"
#define DEFAULT_WEATHER_URL       "weather.opensprinkler.com"

/** Macro define of each option
  * Refer to OpenSprinkler.cpp for details on each option
  */
typedef enum {
  OPTION_FW_VERSION = 0,
  OPTION_TIMEZONE,
  OPTION_USE_NTP,
  OPTION_USE_DHCP,
  OPTION_STATIC_IP1,
  OPTION_STATIC_IP2,
  OPTION_STATIC_IP3,
  OPTION_STATIC_IP4,
  OPTION_GATEWAY_IP1,
  OPTION_GATEWAY_IP2,
  OPTION_GATEWAY_IP3,
  OPTION_GATEWAY_IP4,
  OPTION_HTTPPORT_0,
  OPTION_HTTPPORT_1,
  OPTION_HW_VERSION,
  OPTION_EXT_BOARDS,
  OPTION_SEQUENTIAL_RETIRED,
  OPTION_STATION_DELAY_TIME,
  OPTION_MASTER_STATION,
  OPTION_MASTER_ON_ADJ,
  OPTION_MASTER_OFF_ADJ,
  OPTION_SENSOR_TYPE,
  OPTION_RAINSENSOR_TYPE,
  OPTION_WATER_PERCENTAGE,
  OPTION_DEVICE_ENABLE,
  OPTION_IGNORE_PASSWORD,
  OPTION_DEVICE_ID,
  OPTION_LCD_CONTRAST,
  OPTION_LCD_BACKLIGHT,
  OPTION_LCD_DIMMING,
  OPTION_BOOST_TIME,
  OPTION_USE_WEATHER,
  OPTION_NTP_IP1,
  OPTION_NTP_IP2,
  OPTION_NTP_IP3,
  OPTION_NTP_IP4,
  OPTION_ENABLE_LOGGING,
  OPTION_MASTER_STATION_2,
  OPTION_MASTER_ON_ADJ_2,
  OPTION_MASTER_OFF_ADJ_2,
  OPTION_FW_MINOR,
  OPTION_PULSE_RATE_0,
  OPTION_PULSE_RATE_1,
  OPTION_REMOTE_EXT_MODE,
  OPTION_DNS_IP1,
  OPTION_DNS_IP2,
  OPTION_DNS_IP3,
  OPTION_DNS_IP4,
  OPTION_SPE_AUTO_REFRESH,
  OPTION_IFTTT_ENABLE,
  OPTION_RESET,
  NUM_OPTIONS	// total number of options
} OS_OPTION_t;

/** Log Data Type */
#define LOGDATA_STATION    0x00
#define LOGDATA_RAINSENSE  0x01
#define LOGDATA_RAINDELAY  0x02
#define LOGDATA_WATERLEVEL 0x03
#define LOGDATA_FLOWSENSE  0x04

#undef OS_HW_VERSION

/** Hardware defines */

#define OS_HW_VERSION (OS_HW_VERSION_BASE+22)

#ifndef NORF
  #define PIN_RF_DATA       28    // RF data pin
  #define PORT_RF        PORTA
  #define PINX_RF        PINA3
#endif
#ifndef NOSHIFT
  #define PIN_SR_LATCH       3    // shift register latch pin
  #define PIN_SR_DATA       21    // shift register data pin
  #define PIN_SR_CLOCK      22    // shift register clock pin
  #define PIN_SR_OE          1    // shift register output enable pin
#else
#define PIN_STATIONS_LIST 0,1,2,3,4,5,6,7 // Controlling stations directly from those pins - should match MAX_NUM_STATIONS
#endif

#define PIN_HEARTBEAT      9	 // Heartbeat LED to blink every second
#define PIN_RTC_VCC	   A3
#define PIN_RTC_GND	   A2
#define PIN_ETHER_CS       10    // Ethernet controller chip select pin

#ifndef NOSD
  #define PIN_SD_CS          0    // SD card chip select pin
#endif

#ifndef NORAINSENSOR
#define PIN_RAINSENSOR    A1    // rain sensor is connected to pin D3
#define PIN_FLOWSENSOR    A2    // flow sensor (currently shared with rain sensor, change if using a different pin)
#endif

#ifndef NOFLOWSENSOR
#define PIN_FLOWSENSOR_INT 8    // flow sensor interrupt pin (INT1)
#endif

  // Ethernet buffer size
#define ETHER_BUFFER_SIZE  512   // ATmega328 has 2K RAM, so use a smaller buffer

#ifndef OSMICRO
#define 	wdt_reset()   __asm__ __volatile__ ("wdr")  // watchdog timer reset
#endif

//#define SERIAL_DEBUG
#if defined(SERIAL_DEBUG) /** Serial debug functions */
    #define DEBUG_BEGIN(x)   Serial.begin(x)
    #define DEBUG_PRINT(x)   Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTIP(x) ether.printIp("IP:",x)
  #else
    #define DEBUG_BEGIN(x)   {}
    #define DEBUG_PRINT(x)   {}
    #define DEBUG_PRINTLN(x) {}
    #define DEBUG_PRINTIP(x) {}
#endif

  typedef unsigned char   uint8_t;
  typedef unsigned int    uint16_t;
  typedef int int16_t;
typedef unsigned char byte;
typedef unsigned long ulong;

#endif  // _DEFINES_H
