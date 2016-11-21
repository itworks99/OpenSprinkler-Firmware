/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * OpenSprinkler library header file
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


#ifndef _OPENSPRINKLER_H
#define _OPENSPRINKLER_H

#include <Arduino.h>

#include <WSWire.h>
#include <DS1307RTC.h>
#include <TimeLib.h>

#include <avr/eeprom.h>

#include "EtherCard.h"

#include "defines.h"
#include "utils.h"

/** Non-volatile data */
struct NVConData {
  uint16_t sunrise_time;  // sunrise time (in minutes)
  uint16_t sunset_time;   // sunset time (in minutes)
  uint32_t rd_stop_time;  // rain delay stop time
  uint32_t external_ip;   // external ip
};

/** Station special attribute data */
struct StationSpecialData {
  byte type;
  byte data[STATION_SPECIAL_DATA_SIZE];
};

/** Station data structures - Must fit in STATION_SPECIAL_DATA_SIZE */
struct RFStationData {
  byte on[6];
  byte off[6];
  byte timing[4];
};

struct RemoteStationData {
  byte ip[8];
  byte port[4];
  byte sid[2];
};

/** Volatile controller status bits */
struct ConStatus {
  byte enabled:1;           // operation enable (when set, controller operation is enabled)
  byte rain_delayed:1;      // rain delay bit (when set, rain delay is applied)
  byte rain_sensed:1;       // rain sensor bit (when set, it indicates that rain is detected)
  byte program_busy:1;      // HIGH means a program is being executed currently
  byte has_curr_sense:1;    // HIGH means the controller has a current sensing pin
  byte has_sd:1;            // HIGH means a microSD card is detected
  byte safe_reboot:1;       // HIGH means a safe reboot has been marked
  byte has_hwmac:1;         // has hardware MAC chip
  byte req_ntpsync:1;       // request ntpsync
  byte req_network:1;       // request check network
  byte display_board:3;     // the board that is being displayed onto the lcd
  byte network_fails:3;     // number of network fails
  byte mas:8;               // master station index
  byte mas2:8;              // master2 station index
};

extern const char wtopts_filename[];
extern const char stns_filename[];
extern const char ifkey_filename[];
extern const char op_max[];
extern const char op_json_names[];

class OpenSprinkler {
public:

  // data members

  static NVConData nvdata;
  static ConStatus status;
  static ConStatus old_status;
  static byte nboards, nstations;
  static byte hw_type;           // hardware type

  static byte options[];  // option values, max, name, and flag

  static byte station_bits[];     // station activation bits. each byte corresponds to a board (8 stations)
                                  // first byte-> master controller, second byte-> ext. board 1, and so on

  // variables for time keeping
  static ulong sensor_lasttime;  // time when the last sensor reading is recorded
  static ulong flowcount_time_ms;// time stamp when new flow sensor click is received (in milliseconds)
  static ulong flowcount_rt;     // flow count (for computing real-time flow rate)
  static ulong flowcount_log_start; // starting flow count (for logging)
  static ulong raindelay_start_time;  // time when the most recent rain delay started

  static ulong checkwt_lasttime;      // time when weather was checked
  static ulong checkwt_success_lasttime; // time when weather check was successful
  static byte  weather_update_flag;
  // member functions
  // -- setup
  static void update_dev();   // update software for Linux instances
  static void reboot_dev();   // reboot the microcontroller
  static void begin();        // initialization, must call this function before calling other functions
  static byte start_network();  // initialize network with the given mac and port
#ifndef OSMICRO
  static bool read_hardware_mac();  // read hardware mac address
#endif
  static time_t now_tz();
  // -- station names and attributes
  static void get_station_name(byte sid, char buf[]); // get station name
  static void set_station_name(byte sid, char buf[]); // set station name
#ifndef NORF
  static uint16_t parse_rfstation_code(RFStationData *data, ulong *on, ulong *off); // parse rf code into on/off/time sections
  static void switch_rfstation(RFStationData *data, bool turnon);  // switch rf station
#endif
  static void switch_remotestation(RemoteStationData *data, bool turnon); // switch remote station

  static void station_attrib_bits_save(int addr, byte bits[]); // save station attribute bits to nvm
  static void station_attrib_bits_load(int addr, byte bits[]); // load station attribute bits from nvm
  static byte station_attrib_bits_read(int addr); // read one station attribte byte from nvm

  // -- options and data storeage
  static void nvdata_load();
  static void nvdata_save();

  static void options_setup();
  static void options_load();
  static void options_save();

  static byte password_verify(char *pw);  // verify password

  // -- controller operation
  static void enable();           // enable controller operation
  static void disable();          // disable controller operation, all stations will be closed immediately
  static void raindelay_start();  // start raindelay
  static void raindelay_stop();   // stop rain delay
#ifndef NORAINSENSOR
  static void rainsensor_status();// update rainsensor status
#endif

  static int detect_exp();        // detect the number of expansion boards
  static byte weekday_today();    // returns index of today's weekday (Monday is 0)

  static byte set_station_bit(byte sid, byte value); // set station bit of one station (sid->station index, value->0/1)
  static void switch_special_station(byte sid, byte value); // swtich special station
  static void clear_all_station_bits(); // clear all station bits
  static void apply_all_station_bits(); // apply all station bits (activate/deactive values)

};

#endif  // _OPENSPRINKLER_H
