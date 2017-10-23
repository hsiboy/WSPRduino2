/*
"DS18B20"
Arduino-library supporting communication with a single Maxim DS18B20 temperature sensor in 3-wire setup
(separate power supply and 4.7k pullup resistor)

V1.1 / Copyright (C) 2017, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Required ressources:
 - 1 digital input pin
*/

//################################################################################################################
//includes
//################################################################################################################

#include <DS18B20.h>

//################################################################################################################
//declarations
//################################################################################################################

//################################################################################################################
//global functions
//##############################################################################################################

//################################################################################################################
//private functions
//################################################################################################################

// reads the scratchpad and validates data (CRC-check); returns 1 if ok, otherwise returns 0
uint8_t DS18B20Class::readScratchpad() {
  uint8_t is_ok = 0;
  if(reset()) {
    writeByte(0xCC); // send "skip ROM" command
    writeByte(0xBE); // send "read scratchpad" command
    uint8_t crc = 0x00;
    for(uint8_t i=0; i<8; ++i) {
      spad[i] = readByte();

      // the sequqence for calculating the CRC was derived from Paul Stoffrege's OneWire library
      #if defined(__AVR__)
        crc = _crc_ibutton_update(crc, spad[i]);
      #else
        uint8_t dummy = spad[i], lsb;
        for (uint8_t i = 8; i>0; --i) {
          lsb = (crc^dummy)&0x01;
          crc >>= 1;
          dummy >>= 1;
          if(lsb) crc ^= 0x8C;
        }
      #endif
    }
	
	spad[8] = readByte();
    if(crc == spad[8]) {
      is_ok = 1;
    }
  }

  return is_ok;
}

//################################################################################################################

// writes a byte of data to the DS18B20
void DS18B20Class::outputPulse(const uint16_t WIDTH) {
  digitalWrite(pin, 0);
  pinMode(pin, OUTPUT);
  delayMicroseconds(WIDTH);
  pinMode(pin, INPUT);
}

//################################################################################################################

// writes a bit of data to the DS18B20
void DS18B20Class::writeBit(const uint8_t BIT) {
  if(BIT) {
    outputPulse(5);
    delayMicroseconds(70);
  }
  else {
    outputPulse(75);
  }
  delayMicroseconds(5);
}

//################################################################################################################

// reads a bit of data from the DS18B20
uint8_t DS18B20Class::readBit() {
  outputPulse(3);

  delayMicroseconds(3);
  uint8_t value;
  value = digitalRead(pin);
  delayMicroseconds(80);

  return value;
}

//################################################################################################################

void DS18B20Class::writeByte(uint8_t byte) {
  for(uint8_t i=0; i<8; ++i) {
    writeBit(byte & 1);
    byte >>= 1;
  }
}

//################################################################################################################

// reads a byte of data from the DS18B20
uint8_t DS18B20Class::readByte() {
  uint8_t dummy = 0;
  for(uint8_t i=0; i<8; ++i) {
    dummy += (readBit()<<i);
  }

  return dummy;
}

//################################################################################################################

/*
resets the DS18B20
returns 1 if successful, otherwise 0
*/
uint8_t DS18B20Class::reset() {
  outputPulse(600);

  uint8_t ready = 0;
  for(uint8_t i=0; i<20; ++i) {
    delayMicroseconds(20);
    if(!digitalRead(pin)) {
      ready = 1;
    }
  }

  return ready;
}

//################################################################################################################
//public functions
//################################################################################################################

/*
initializes the library; resolution can be 9...12 bit corrseponding to 0.5°C ... 0.0625°C
returns 1 if successful, otherwise 0
*/
uint8_t DS18B20Class::init(const uint8_t PIN, const uint8_t RESOLUTION) {
  if(!is_initialized && RESOLUTION>8 && RESOLUTION<13) {
    pin = PIN;
	pinMode(pin, INPUT);
    if(reset()) {
      uint8_t res = 0x1F + ((RESOLUTION-9)<<5);
      writeByte(0xCC); // send "skip ROM" command
      writeByte(0x4E); // send "write scratchpad" command
      writeByte(0x7D); // send "Th" = 125 degree C
      writeByte(0xC9); // send "Tl" = -55 degree C
      writeByte(res); // send "resolution" = RESOLUTION
      if(readScratchpad() && spad[4]==res) {
        is_initialized = 1;
      }
    }
  }

  return is_initialized;
}

//################################################################################################################

/*
initiates temperature conversion; wait > 93.75*2^(RESOLUTION - 9) ms before attempt reading T
if "IS_STALLING" is True, the function will stall until conversion is done (timeout = 800ms)
returns 1 if successful, otherwise returns 0
*/
uint8_t DS18B20Class::convertTemp(const uint8_t IS_STALLING) {
  uint8_t is_ok = 0;
  if(reset()) {
    is_ok = 1;
    writeByte(0xCC); // send "skip ROM" command
    writeByte(0x44); // send "convert T" command

    if(IS_STALLING) {
      for(uint8_t i=0; i<16; ++i) {
        delay(50);
        is_ok = readBit();
        if(is_ok) { break; }
      }
    }
  }

  return is_ok;
}

//################################################################################################################

/*
reads the temperature; returns actual temperature*16 as an int16_t or 0x8000==-32768 in case of failure
*/
int16_t DS18B20Class::getTemp() {
  int16_t dummy = 0x8000; // pre-set to 0x8000
  if(readScratchpad()) {
    dummy = spad[1];
    dummy <<= 8;
    dummy += spad[0];
  }

  return dummy;
}

DS18B20Class TS;
