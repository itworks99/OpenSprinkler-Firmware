/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * Utility functions header file
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

#ifndef _UTILS_H
#define _UTILS_H

#include "defines.h"

void strncpy_P0(char* dest, const char* src, int n);
byte strcmp_to_nvm(const char* src, int addr);
ulong water_time_resolve(uint16_t v);
byte water_time_encode_signed(int16_t i);
int16_t water_time_decode_signed(byte i);
void write_to_file(const char *name, const char *data, int size, int pos=0, bool trunc=true);
bool read_from_file(const char *name, char *data, int maxsize=TMP_BUFFER_SIZE, int pos=0);
void remove_file(const char *name);

  #define nvm_read_block  eeprom_read_block
  #define nvm_write_block eeprom_write_block
  #define nvm_read_byte   eeprom_read_byte
  #define nvm_write_byte  eeprom_write_byte

#endif // _UTILS_H
