/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * OpenSprinkler library
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

#include "OpenSprinkler.h"

/** Declare static data members */
NVConData OpenSprinkler::nvdata;
ConStatus OpenSprinkler::status;
ConStatus OpenSprinkler::old_status;
byte OpenSprinkler::hw_type;

byte OpenSprinkler::nboards;
byte OpenSprinkler::nstations;
byte OpenSprinkler::station_bits[MAX_EXT_BOARDS+1];

ulong OpenSprinkler::sensor_lasttime;
ulong OpenSprinkler::flowcount_log_start;
ulong OpenSprinkler::flowcount_rt;
ulong OpenSprinkler::flowcount_time_ms;
ulong OpenSprinkler::raindelay_start_time;

ulong OpenSprinkler::checkwt_lasttime;
ulong OpenSprinkler::checkwt_success_lasttime;
byte OpenSprinkler::weather_update_flag;

char tmp_buffer[TMP_BUFFER_SIZE+1];       // scratch buffer

  const char wtopts_filename[] PROGMEM = WEATHER_OPTS_FILENAME;
  const char stns_filename[]   PROGMEM = STATION_ATTR_FILENAME;
#ifndef NOSD
#include "SdFat.h"
extern SdFat sd;
#endif

//const char ifkey_filename[] PROGMEM = IFTTT_KEY_FILENAME;

#ifdef NOSHIFT
// station pins setup
byte stationPins[MAX_NUM_STATIONS] =
    {
    PIN_STATIONS_LIST
    };
#endif

/** Option json names (stored in progmem) */
// IMPORTANT: each json name is strictly 5 characters
// with 0 fillings if less
#define OP_JSON_NAME_STEPSIZE 5

const char op_json_names[] PROGMEM =
    "fwv\0\0"
    "tz\0\0\0"
    "ntp\0\0"
    "dhcp\0"
    "ip1\0\0"
    "ip2\0\0"
    "ip3\0\0"
    "ip4\0\0"
    "gw1\0\0"
    "gw2\0\0"
    "gw3\0\0"
    "gw4\0\0"
    "hp0\0\0"
    "hp1\0\0"
    "hwv\0\0"
    "ext\0\0"
    "seq\0\0"
    "sdt\0\0"
    "mas\0\0"
    "mton\0"
    "mtof\0"
    "urs\0\0"
    "rso\0\0"
    "wl\0\0\0"
    "den\0\0"
    "ipas\0"
    "devid"
    "con\0\0"
    "lit\0\0"
    "dim\0\0"
    "bst\0\0"
    "uwt\0\0"
    "ntp1\0"
    "ntp2\0"
    "ntp3\0"
    "ntp4\0"
    "lg\0\0\0"
    "mas2\0"
    "mton2"
    "mtof2"
    "fwm\0\0"
    "fpr0\0"
    "fpr1\0"
    "re\0\0\0"
    "dns1\0"
    "dns2\0"
    "dns3\0"
    "dns4\0"
    "sar\0\0"
    "ife\0\0"
    "reset";

/** Option maximum values (stored in progmem) */
const char op_max[] PROGMEM = {
  0,
  108,
  1,
  1,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  255,
  0,
  MAX_EXT_BOARDS,
  1,
  255,
  MAX_NUM_STATIONS,
  255,
  255,
  255,
  1,
  250,
  1,
  1,
  255,
  255,
  255,
  255,
  250,
  255,
  255,
  255,
  255,
  255,
  1,
  MAX_NUM_STATIONS,
  255,
  255,
  0,
  255,
  255,
  1,
  255,
  255,
  255,
  255,
  1,
  255,
  1
};

/** Option values (stored in RAM) */
byte OpenSprinkler::options[] = {
  OS_FW_VERSION, // firmware version
    28, // default time zone: GMT-5
  1,  // 0: disable NTP sync, 1: enable NTP sync
  1,  // 0: use static ip, 1: use dhcp
  0,  // this and next 3 bytes define static ip
  0,
  0,
  0,
  0,  // this and next 3 bytes define static gateway ip
  0,
  0,
  0,
    80, // on AVR, the default HTTP port is 80 - this and next byte define http port number
  0,
  OS_HW_VERSION,
  0,  // number of 8-station extension board. 0: no extension boards
  1,  // the option 'sequential' is now retired
  120,// station delay time (-10 minutes to 10 minutes).
  0,  // index of master station. 0: no master station
  120,// master on time adjusted time (-10 minutes to 10 minutes)
  120,// master off adjusted time (-10 minutes to 10 minutes)
  0,  // sensor function (see SENSOR_TYPE macro defines)
  0,  // rain sensor type. 0: normally closed; 1: normally open.
  100,// water level (default 100%),
  1,  // device enable
  0,  // 1: ignore password; 0: use password
  0,  // device id
    0,  // lcd contrast
    0,  // lcd backlight
    0, // lcd dimming
  80, // boost time (only valid to DC and LATCH type)
  0,  // weather algorithm (0 means not using weather algorithm)
  50, // this and the next three bytes define the ntp server ip
  97,
  210,
  169,
    0,  // enable logging: 0: disable; 1: enable.
  0,  // index of master2. 0: no master2 station
  120,// master2 on adjusted time
  120,// master2 off adjusted time
  OS_FW_MINOR, // firmware minor version
  100,// this and next byte define flow pulse rate (100x)
  0,  // default is 1.00 (100)
  0,  // set as remote extension
  8,  // this and the next three bytes define the custom dns server ip
  8,
  8,
  8,
  0,  // special station auto refresh
  0,  // ifttt enable bits
  0   // reset
};

/** Calculate local time (UTC time plus time zone offset) */
time_t OpenSprinkler::now_tz() {
  return now()+(int32_t)3600/4*(int32_t)(options[OPTION_TIMEZONE]-48);
}

// AVR network init functions

/** read hardware MAC */
#ifndef OSMICRO
#define MAC_CTRL_ID 0x50

bool OpenSprinkler::read_hardware_mac() {
  uint8_t ret;
  Wire.beginTransmission(MAC_CTRL_ID);
  Wire.write((uint8_t)(0x00));
  ret = Wire.endTransmission();
  if (ret)  return false;

  Wire.beginTransmission(MAC_CTRL_ID);
  Wire.write(0xFA); // The address of the register we want
  Wire.endTransmission(); // Send the data
  Wire.requestFrom(MAC_CTRL_ID, 6); // Request 6 bytes from the EEPROM
  while (!Wire.available()); // Wait for the response
  for (ret=0;ret<6;ret++) {
    tmp_buffer[ret] = Wire.read();
  }
  return true;
    }
#endif

void(* resetFunc) (void) = 0; // AVR software reset function

/** Initialize network with the given mac address and http port */
byte OpenSprinkler::start_network() {

#ifndef OSMICRO
  // new from 2.2: read hardware MAC
  if(!read_hardware_mac())
  {
    // if no hardware MAC exists, use software MAC
    tmp_buffer[0] = 0x00;
    tmp_buffer[1] = 0x69;
    tmp_buffer[2] = 0x69;
    tmp_buffer[3] = 0x2D;
    tmp_buffer[4] = 0x31;
    tmp_buffer[5] = options[OPTION_DEVICE_ID];
  } else {
    // has hardware MAC chip
    status.has_hwmac = 1;
  }
#else
    // use software MAC
    tmp_buffer[0] = 0x00;
    tmp_buffer[1] = 0x69;
    tmp_buffer[2] = 0x69;
    tmp_buffer[3] = 0x2D;
    tmp_buffer[4] = 0x31;
    tmp_buffer[5] = options[OPTION_DEVICE_ID];
#endif
  if(!ether.begin(ETHER_BUFFER_SIZE, (uint8_t*)tmp_buffer, PIN_ETHER_CS))  return 0;
  // calculate http port number
  ether.hisport = (unsigned int)(options[OPTION_HTTPPORT_1]<<8) + (unsigned int)options[OPTION_HTTPPORT_0];

  if (options[OPTION_USE_DHCP]) {
    // set up DHCP
    // register with domain name "OS-xx" where xx is the last byte of the MAC address
    if (!ether.dhcpSetup()) return 0;
    // once we have valid DHCP IP, we write these into static IP / gateway IP
    memcpy(options+OPTION_STATIC_IP1, ether.myip, 4);
    memcpy(options+OPTION_GATEWAY_IP1, ether.gwip,4);
    memcpy(options+OPTION_DNS_IP1, ether.dnsip, 4);
    options_save();

  } else {
    // set up static IP
    byte *staticip = options+OPTION_STATIC_IP1;
    byte *gateway  = options+OPTION_GATEWAY_IP1;
    byte *dns      = options+OPTION_DNS_IP1;
    if (!ether.staticSetup(staticip, gateway, dns))  return 0;
  }
  return 1;
}

/** Reboot controller */
void OpenSprinkler::reboot_dev() {
  resetFunc();
}

#ifndef NOFLOWSENSOR
extern void flow_isr();
#endif

/** Initialize pins, controller variables, LCD */
void OpenSprinkler::begin() {

#ifndef NOSHIFT
  // shift register setup
  pinMode(PIN_SR_OE, OUTPUT);
  // pull shift register OE high to disable output
  digitalWrite(PIN_SR_OE, HIGH);
  pinMode(PIN_SR_LATCH, OUTPUT);
  digitalWrite(PIN_SR_LATCH, HIGH);
  pinMode(PIN_SR_CLOCK, OUTPUT);
  pinMode(PIN_SR_DATA,  OUTPUT);
#else

    for (byte i = 0; i < MAX_NUM_STATIONS; i++)
	{
	pinMode(stationPins[i], OUTPUT);
	digitalWrite(stationPins[i], LOW);
	}

#endif
	// Reset all stations
  clear_all_station_bits();
  apply_all_station_bits();

#ifndef NOSHIFT
  // pull shift register OE low to enable output
  digitalWrite(PIN_SR_OE, LOW);
#endif
#ifndef NORAINSENSOR
  // Rain sensor port set up
  pinMode(PIN_RAINSENSOR, INPUT);
  // Set up sensors
  digitalWrite(PIN_RAINSENSOR, HIGH); // enabled internal pullup on rain sensor
#endif

#ifndef NOFLOWSENSOR
  attachInterrupt(PIN_FLOWSENSOR_INT, flow_isr, FALLING);
#endif

  // Default controller status variables
  // Static variables are assigned 0 by default
  // so only need to initialize non-zero ones
  status.enabled = 1;
  status.safe_reboot = 0;

  old_status = status;

  nvdata.sunrise_time = 360;  // 6:00am default sunrise
  nvdata.sunset_time = 1080;  // 6:00pm default sunset

  nboards = 1;
  nstations = 8;

#ifndef NORF
  // set rf data pin
  pinMode(PIN_RF_DATA, OUTPUT);
  digitalWrite(PIN_RF_DATA, LOW);
#endif

    hw_type = HW_TYPE_UNIV;

// AVR SD and LCD functions
  // Init I2C
    // Wire.begin();

#ifndef NOSD
  // set sd cs pin high to release SD
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  if(sd.begin(PIN_SD_CS, SPI_HALF_SPEED)) {
    status.has_sd = 1;
  }
#else
    status.has_sd = 0;
#endif
}

/** Apply all station bits
 * !!! This will activate/deactivate valves !!!
 */
void OpenSprinkler::apply_all_station_bits() {
#ifndef NOSHIFT
    digitalWrite(PIN_SR_LATCH, LOW);
#endif
    byte bid, s, sbits;

  // Shift out all station bit values
  // from the highest bit to the lowest
  for(bid=0;bid<=MAX_EXT_BOARDS;bid++) {
    if (status.enabled)
      sbits = station_bits[MAX_EXT_BOARDS-bid];
    else
      sbits = 0;
#ifndef NOSHIFT
    for(s=0;s<8;s++) {
      digitalWrite(PIN_SR_CLOCK, LOW);
      digitalWrite(PIN_SR_DATA, (sbits & ((byte)1<<(7-s))) ? HIGH : LOW );
      digitalWrite(PIN_SR_CLOCK, HIGH);
    }
#else
	for (s = 0; s < MAX_NUM_STATIONS; s++)
	    {
	    digitalWrite(
			 stationPins[s],
			 (sbits & ((byte) 1 << (7 - s))) ? HIGH : LOW); // Stations are connected to digital pins as defined by PIN_STATIONS_LIST
	    delay(10); //Little delay to let relay switch completely (5 to 10ms is required)
	    }
#endif
	}
#ifndef NOSHIFT
    digitalWrite(PIN_SR_LATCH, HIGH);
#endif

  if(options[OPTION_SPE_AUTO_REFRESH]) {
    // handle refresh of RF and remote stations
    // each time apply_all_station_bits is called
    // we refresh the station whose index is the current time modulo MAX_NUM_STATIONS
    static byte last_sid = 0;
    byte sid = now() % MAX_NUM_STATIONS;
    if (sid != last_sid) {  // avoid refreshing the same station twice in a roll
      last_sid = sid;
      bid=sid>>3;
      s=sid&0x07;
      switch_special_station(sid, (station_bits[bid]>>s)&0x01);
    }
  }
}
#ifndef NORAINSENSOR
/** Read rain sensor status */
void OpenSprinkler::rainsensor_status() {
  // options[OPTION_RS_TYPE]: 0 if normally closed, 1 if normally open
  if(options[OPTION_SENSOR_TYPE]!=SENSOR_TYPE_RAIN) return;
  status.rain_sensed = (digitalRead(PIN_RAINSENSOR) == options[OPTION_RAINSENSOR_TYPE] ? 0 : 1);
    }
#endif

/** Read the number of 8-station expansion boards */
// AVR has capability to detect number of expansion boards
int OpenSprinkler::detect_exp() {
  return -1;
}
#ifndef NORF
/** Convert hex code to ulong integer */
static ulong hex2ulong(byte *code, byte len) {
  char c;
  ulong v = 0;
  for(byte i=0;i<len;i++) {
    c = code[i];
    v <<= 4;
    if(c>='0' && c<='9') {
      v += (c-'0');
    } else if (c>='A' && c<='F') {
      v += 10 + (c-'A');
    } else if (c>='a' && c<='f') {
      v += 10 + (c-'a');
    } else {
      return 0;
    }
  }
  return v;
}

/** Parse RF code into on/off/timeing sections */
uint16_t OpenSprinkler::parse_rfstation_code(RFStationData *data, ulong* on, ulong *off) {

  ulong v;
  v = hex2ulong(data->on, sizeof(data->on));
  if (!v) return 0;
  if (on) *on = v;
	v = hex2ulong(data->off, sizeof(data->off));
  if (!v) return 0;
  if (off) *off = v;
	v = hex2ulong(data->timing, sizeof(data->timing));
  if (!v) return 0;
  return v;
    }
#endif
/** Get station name from NVM */
void OpenSprinkler::get_station_name(byte sid, char tmp[]) {
  tmp[STATION_NAME_SIZE]=0;
  nvm_read_block(tmp, (void*)(ADDR_NVM_STN_NAMES+(int)sid*STATION_NAME_SIZE), STATION_NAME_SIZE);
}

/** Set station name to NVM */
void OpenSprinkler::set_station_name(byte sid, char tmp[]) {
  tmp[STATION_NAME_SIZE]=0;
  nvm_write_block(tmp, (void*)(ADDR_NVM_STN_NAMES+(int)sid*STATION_NAME_SIZE), STATION_NAME_SIZE);
}

/** Save station attribute bits to NVM */
void OpenSprinkler::station_attrib_bits_save(int addr, byte bits[]) {
  nvm_write_block(bits, (void*)addr, MAX_EXT_BOARDS+1);
}

/** Load all station attribute bits from NVM */
void OpenSprinkler::station_attrib_bits_load(int addr, byte bits[]) {
  nvm_read_block(bits, (void*)addr, MAX_EXT_BOARDS+1);
}

/** Read one station attribute byte from NVM */
byte OpenSprinkler::station_attrib_bits_read(int addr) {
  return nvm_read_byte((byte*)addr);
}

/** verify if a string matches password */
byte OpenSprinkler::password_verify(char *pw) {
  byte *addr = (byte*)ADDR_NVM_PASSWORD;
  byte c1, c2;
  while(1) {
    if(addr == (byte*)ADDR_NVM_PASSWORD+MAX_USER_PASSWORD)
      c1 = 0;
    else
      c1 = nvm_read_byte(addr++);
    c2 = *pw++;
    if (c1==0 || c2==0)
      break;
    if (c1!=c2) {
      return 0;
    }
  }
  return (c1==c2) ? 1 : 0;
}

// ==================
// Schedule Functions
// ==================

/** Index of today's weekday (Monday is 0) */
byte OpenSprinkler::weekday_today() {
  //return ((byte)weekday()+5)%7; // Time::weekday() assumes Sunday is 1

  ulong wd = now_tz() / 86400L;
  return (wd+3) % 7;  // Jan 1, 1970 is a Thursday

}

/** Switch special station */
void OpenSprinkler::switch_special_station(byte sid, byte value) {
  // check station special bit
  if(station_attrib_bits_read(ADDR_NVM_STNSPE+(sid>>3))&(1<<(sid&0x07))) {
    // read station special data from sd card
    int stepsize=sizeof(StationSpecialData);
    read_from_file(stns_filename, tmp_buffer, stepsize, sid*stepsize);
    StationSpecialData *stn = (StationSpecialData *)tmp_buffer;
    // check station type
#ifndef NORF
    if(stn->type==STN_TYPE_RF) {
      // transmit RF signal
      switch_rfstation((RFStationData *)stn->data, value);
	    }
	else
#endif
	if (stn->type == STN_TYPE_REMOTE)
	    {
      // request remote station
      switch_remotestation((RemoteStationData *)stn->data, value);
    }

  }
}

/** Set station bit
 * This function sets/resets the corresponding station bit variable
 * You have to call apply_all_station_bits next to apply the bits
 * (which results in physical actions of opening/closing valves).
 */
byte OpenSprinkler::set_station_bit(byte sid, byte value) {
  byte *data = station_bits+(sid>>3);  // pointer to the station byte
  byte mask = (byte)1<<(sid&0x07); // mask
  if (value) {
    if((*data)&mask) return 0;  // if bit is already set, return no change
    else {
      (*data) = (*data) | mask;

      switch_special_station(sid, 1); // handle special stations
      return 1;
    }
  } else {
    if(!((*data)&mask)) return 0; // if bit is already reset, return no change
    else {
      (*data) = (*data) & (~mask);
      switch_special_station(sid, 0); // handle special stations
      return 255;
    }
  }
  return 0;
}

/** Clear all station bits */
void OpenSprinkler::clear_all_station_bits() {
  byte sid;
  for(sid=0;sid<=MAX_NUM_STATIONS;sid++) {
    set_station_bit(sid, 0);
  }
}
#ifndef NORF
/** Transmit one RF signal bit */
void transmit_rfbit(ulong lenH, ulong lenL) {
  PORT_RF |= (1<<PINX_RF);
  delayMicroseconds(lenH);
  PORT_RF &=~(1<<PINX_RF);
  delayMicroseconds(lenL);
    }

/** Transmit RF signal */
void send_rfsignal(ulong code, ulong len) {

    ulong len3 = len * 3;
  ulong len31 = len * 31;
  for(byte n=0;n<15;n++) {
    int i=23;
    // send code
    while(i>=0) {
      if ((code>>i) & 1) {
        transmit_rfbit(len3, len);
      } else {
        transmit_rfbit(len, len3);
      }
      i--;
    };
    // send sync
    transmit_rfbit(len, len31);
  }

}

/** Switch RF station
 * This function takes a RF code,
 * parses it into signals and timing,
 * and sends it out through RF transmitter.
 */
void OpenSprinkler::switch_rfstation(RFStationData *data, bool turnon) {

    ulong on, off;
  uint16_t length = parse_rfstation_code(data, &on, &off);
  send_rfsignal(turnon ? on : off, length);

    }
#endif

/** Callback function for browseUrl calls */
void httpget_callback(byte status, uint16_t off, uint16_t len) {
#if defined(SERIAL_DEBUG)
  Ethernet::buffer[off+ETHER_BUFFER_SIZE-1] = 0;
  DEBUG_PRINTLN((const char*) Ethernet::buffer + off);
#endif
}

/** Switch remote station
 * This function takes a remote station code,
 * parses it into remote IP, port, station index,
 * and makes a HTTP GET request.
 * The remote controller is assumed to have the same
 * password as the main controller
 */
void OpenSprinkler::switch_remotestation(RemoteStationData *data, bool turnon) {
#ifndef NORF
  // construct string
  ulong ip = hex2ulong(data->ip, sizeof(data->ip));
  ether.hisip[0] = ip>>24;
  ether.hisip[1] = (ip>>16)&0xff;
  ether.hisip[2] = (ip>>8)&0xff;
  ether.hisip[3] = ip&0xff;

  uint16_t _port = ether.hisport; // save current port number
  ether.hisport = hex2ulong(data->port, sizeof(data->port));

  char *p = tmp_buffer + sizeof(RemoteStationData) + 1;
  BufferFiller bf = (byte*)p;
  // MAX_NUM_STATIONS is the refresh cycle
  uint16_t timer = options[OPTION_SPE_AUTO_REFRESH]?2*MAX_NUM_STATIONS:65535;
  bf.emit_p(PSTR("?pw=$E&sid=$D&en=$D&t=$D"),
            ADDR_NVM_PASSWORD,
            (int)hex2ulong(data->sid,sizeof(data->sid)),
            turnon, timer);
  ether.browseUrl(PSTR("/cm"), p, PSTR("*"), httpget_callback);
  for(int l=0;l<100;l++)  ether.packetLoop(ether.packetReceive());
  ether.hisport = _port;
#endif
    }

/** Setup function for options */
void OpenSprinkler::options_setup() {

  // add 0.25 second delay to allow nvm to stablize
  delay(250);

  byte curr_ver = nvm_read_byte((byte*)(ADDR_NVM_OPTIONS+OPTION_FW_VERSION));

  // check reset condition: either firmware version has changed, or reset flag is up
  // if so, trigger a factory reset
  if (curr_ver != OS_FW_VERSION || nvm_read_byte((byte*)(ADDR_NVM_OPTIONS+OPTION_RESET))==0xAA)  {

    // ======== Reset NVM data ========
    int i, sn;

    // 0. wipe out nvm
    for(i=0;i<TMP_BUFFER_SIZE;i++) tmp_buffer[i]=0;
    for(i=0;i<NVM_SIZE;i+=TMP_BUFFER_SIZE) {
      int nbytes = ((NVM_SIZE-i)>TMP_BUFFER_SIZE)?TMP_BUFFER_SIZE:(NVM_SIZE-i);
      nvm_write_block(tmp_buffer, (void*)i, nbytes);
    }

    // 1. write non-volatile controller status
    nvdata_save();

    // 2. write string parameters
    nvm_write_block(DEFAULT_PASSWORD, (void*)ADDR_NVM_PASSWORD, strlen(DEFAULT_PASSWORD)+1);
    nvm_write_block(DEFAULT_LOCATION, (void*)ADDR_NVM_LOCATION, strlen(DEFAULT_LOCATION)+1);
    nvm_write_block(DEFAULT_JAVASCRIPT_URL, (void*)ADDR_NVM_JAVASCRIPTURL, strlen(DEFAULT_JAVASCRIPT_URL)+1);
    nvm_write_block(DEFAULT_WEATHER_URL, (void*)ADDR_NVM_WEATHERURL, strlen(DEFAULT_WEATHER_URL)+1);
    nvm_write_block(DEFAULT_WEATHER_KEY, (void*)ADDR_NVM_WEATHER_KEY, strlen(DEFAULT_WEATHER_KEY)+1);

    // 3. reset station names and special attributes, default Sxx
    tmp_buffer[0]='S';
    tmp_buffer[3]=0;
    for(i=ADDR_NVM_STN_NAMES, sn=1; i<ADDR_NVM_MAS_OP; i+=STATION_NAME_SIZE, sn++) {
      tmp_buffer[1]='0'+(sn/10);
      tmp_buffer[2]='0'+(sn%10);
      nvm_write_block(tmp_buffer, (void*)i, strlen(tmp_buffer)+1);
    }

    remove_file(stns_filename);
    tmp_buffer[0]=STN_TYPE_STANDARD;
    tmp_buffer[1]='0';
    tmp_buffer[2]=0;
    int stepsize=sizeof(StationSpecialData);
    for(i=0;i<MAX_NUM_STATIONS;i++) {
        write_to_file(stns_filename, tmp_buffer, stepsize, i*stepsize, false);
    }
    // 4. reset station attribute bits
    // since we wiped out nvm, only non-zero attributes need to be initialized
    for(i=0;i<MAX_EXT_BOARDS+1;i++) {
      tmp_buffer[i]=0xff;
    }
    nvm_write_block(tmp_buffer, (void*)ADDR_NVM_MAS_OP, MAX_EXT_BOARDS+1);
    nvm_write_block(tmp_buffer, (void*)ADDR_NVM_STNSEQ, MAX_EXT_BOARDS+1);

    // 5. delete sd file
    remove_file(wtopts_filename);

    // 6. write options
    options_save(); // write default option values

    //======== END OF NVM RESET CODE ========

    // restart after resetting NVM.
    delay(500);

    reboot_dev();

  }
    else
  {
    // load ram parameters from nvm
    // load options
    options_load();
    // load non-volatile controller data
    nvdata_load();
  }
}

/** Load non-volatile controller status data from internal NVM */
void OpenSprinkler::nvdata_load() {
  nvm_read_block(&nvdata, (void*)ADDR_NVM_NVCONDATA, sizeof(NVConData));
  old_status = status;
}

/** Save non-volatile controller status data to internal NVM */
void OpenSprinkler::nvdata_save() {
  nvm_write_block(&nvdata, (void*)ADDR_NVM_NVCONDATA, sizeof(NVConData));
}

/** Load options from internal NVM */
void OpenSprinkler::options_load() {
  nvm_read_block(tmp_buffer, (void*)ADDR_NVM_OPTIONS, NUM_OPTIONS);
  for (byte i=0; i<NUM_OPTIONS; i++) {
    options[i] = tmp_buffer[i];
  }
  nboards = options[OPTION_EXT_BOARDS]+1;
  nstations = nboards * 8;
  status.enabled = options[OPTION_DEVICE_ENABLE];
  options[OPTION_FW_MINOR] = OS_FW_MINOR;
}

/** Save options to internal NVM */
void OpenSprinkler::options_save() {
  // save options in reverse order so version number is written the last
  for (int i=NUM_OPTIONS-1; i>=0; i--) {
    tmp_buffer[i] = options[i];
  }
  nvm_write_block(tmp_buffer, (void*)ADDR_NVM_OPTIONS, NUM_OPTIONS);
  nboards = options[OPTION_EXT_BOARDS]+1;
  nstations = nboards * 8;
  status.enabled = options[OPTION_DEVICE_ENABLE];
}

// ==============================
// Controller Operation Functions
// ==============================

/** Enable controller operation */
void OpenSprinkler::enable() {
  status.enabled = 1;
  options[OPTION_DEVICE_ENABLE] = 1;
  options_save();
}

/** Disable controller operation */
void OpenSprinkler::disable() {
  status.enabled = 0;
  options[OPTION_DEVICE_ENABLE] = 0;
  options_save();
}

/** Start rain delay */
void OpenSprinkler::raindelay_start() {
  status.rain_delayed = 1;
  nvdata_save();
}

/** Stop rain delay */
void OpenSprinkler::raindelay_stop() {
  status.rain_delayed = 0;
  nvdata.rd_stop_time = 0;
  nvdata_save();
}

