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

#ifndef DS18B20_h_
#define DS18B20_h_

#if (ARDUINO >= 100)
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

#include <stdint.h>

#if defined(__AVR__)
  #include <util/crc16.h>
#endif

//################################################################################################################
//definitions
//################################################################################################################

//################################################################################################################

class DS18B20Class {

public:

  uint8_t init(const uint8_t PIN, const uint8_t RESOLUTION);
  uint8_t convertTemp(const uint8_t IS_STALLING);
  int16_t getTemp();

private:

  // flag holding the init-state of this routine
  uint8_t is_initialized = 0;
  // the data-port
  uint8_t pin;
  // the scratchpad-content (including CRC)
  uint8_t spad[9];

  uint8_t readScratchpad();
  void outputPulse(const uint16_t WIDTH);
  void writeBit(const uint8_t BIT);
  void writeByte(uint8_t byte);
  uint8_t readBit();
  uint8_t readByte();
  uint8_t reset();

};

extern DS18B20Class TS;

#endif // DS18B20_h_
