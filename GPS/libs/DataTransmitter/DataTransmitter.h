/*
  "DataTransmitter"
  Library supporting wireless or wire-bound serial data broadcasting
  (e.g. sensor data via a 2-wire connection or radio broadcast on an ISM-frequency)
  The transmission is DC-free (Manchester-coded) and error-checked (CRC8)
  Requires library "CRCGenerator"
  
  V1.0 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.

  Required ressources:
   - 1 digital output pin
   - Timer 2 (conflicting with the functions  "analogWrite()" at pins 3 & 11 and "tone()" (all pins)).
*/

#ifndef DataTransmitter_h_
#define DataTransmitter_h_

#if (ARDUINO >= 100)
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

#include <stdint.h>
#include <CRCGenerator.h>

//################################################################################################################
//definitions
//################################################################################################################

// structure containing all relevant parameter of a dataset
struct dset_t {
  uint8_t* data; // a pointer to the actual data array
  uint8_t ID; // the ID of the dataset
  uint8_t SIZE; // number of elements (1...255 Byte)
};

//################################################################################################################

class DataTransmitterClass {

public:

  void init(const uint8_t BYTE_RATE, const uint8_t TRANSMITTER_PIN, const uint8_t IS_LOW_ACTIVE);
  dset_t createDataset(const void* X, const uint8_t ID, const uint8_t SCOPE);
  uint8_t dataTransfer(const void* X, const uint16_t SCOPE, const dset_t DSET, const uint8_t POS,
                       const uint8_t WRITE);
  uint8_t transmitData(const dset_t DSET);

  void isr(); // For internal use only / do not call!!!

private:

  uint8_t pin;
  uint8_t* data;
  volatile uint8_t bytes = 0;
  uint16_t byte_counter;
  uint8_t step;
  uint8_t prev_symbol;
  uint8_t curr_symbol;
  uint32_t curr_byte;
  uint8_t block_size;
  uint8_t pin_value = 0;

  uint8_t prescaler = 0;

};

extern DataTransmitterClass DT;

#endif // DataTransmitter_h_
