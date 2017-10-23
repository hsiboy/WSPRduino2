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

//################################################################################################################
//includes
//################################################################################################################

#include <DataTransmitter.h>

//################################################################################################################
//declarations
//################################################################################################################

//################################################################################################################
//global functions
//##############################################################################################################

// timer 2 compa-isr
ISR(TIMER2_COMPA_vect) {
  DT.isr();
}

//################################################################################################################
//private functions
//##############################################################################################################

//################################################################################################################
//public functions
//################################################################################################################

// performs Manchester-coded transmission of dataset (LSB first)
void DataTransmitterClass::isr() {

  if(step < 4) {
    pin_value = !pin_value;
  }
  else {
    // each symbol (bit) requires two steps to transmit
    if(!(step & 1)) {
      curr_symbol = curr_byte & 1;
      curr_byte >>= 1;
      // toggle output if symbol-value (bit-value) has changed
      if(prev_symbol^curr_symbol) {
        pin_value = !pin_value;
      }
      prev_symbol = curr_symbol;
    }

    if(step==block_size) {
      step = 3;
      if(byte_counter == bytes) {
        if(block_size == 4) {
          // stop timer 2 and set output to 0
          TCCR2B = 0; // set entire TCCR2B register to 0
          bytes = 0;
        }
        else {
          curr_byte = 0;
          block_size = 4;
        }
      }
      else {
        curr_byte = *(data+byte_counter);
        block_size = 19;
        ++byte_counter;
      }
    }
  }

  ++step;

  digitalWrite(pin, pin_value = !pin_value);

}

//################################################################################################################

/*
initializes the library

"BYTE_RATE" -> the target gross data transfer rate in Byte/s (the algorithm will attempt getting as close as possible
 to this number). Valid entries are 4...255. A small overhead is introduced by the system (2 Byte for CRC & ID
 + 3.5 bit for the preamble)
"TRANSMITTER_PIN" -> the digital pin the transmitter is connected to
"IS_LOW_ACTIVE" -> if True, the transmitter-logic is low-active (low->TX on)
*/
void DataTransmitterClass::init(const uint8_t BYTE_RATE, const uint8_t TRANSMITTER_PIN, const uint8_t IS_LOW_ACTIVE) {

  pin = TRANSMITTER_PIN;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, IS_LOW_ACTIVE);

  // ... the Timer 2 setup ...

  // calculate time multiplier for 16MHz clock from given Byte-rate
  const uint8_t PRE[7] = {0, 3, 2, 1, 1, 1, 2};
  uint8_t i = 0;
  uint32_t multiplier = 1000000/max(BYTE_RATE, 4);
  // determine prescaler and calculate OCR2A-value
  do {
    multiplier>>=PRE[i];
    ++i;
  }
  while(multiplier > 0x100);
  prescaler = i;
  --multiplier;
  
  cli(); // disable global interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // set entire TCCR2B register to 0
  OCR2A = multiplier; // set compare match register depending on the calculated trigger time
  TCCR2A |= (1 << WGM21); // enable CTC mode
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
  sei(); // enable global interrupts
}

//################################################################################################################

/*
creates a structure of the type dset_t (transmitter dataset parameter)

"X" -> a pointer to the data-element (can be a single variable, array, ...)
"ID" -> the ID of the dataset (0...255)
"SCOPE" -> the number of data elements times their size (1...255 Byte)
*/

dset_t DataTransmitterClass::createDataset(const void* X, const uint8_t ID, const uint8_t SCOPE) {
  dset_t dummy;

  dummy.data = (uint8_t*) X;
  dummy.ID = ID;
  dummy.SIZE = SCOPE;

  return dummy;
}

//################################################################################################################

/*
transmits a set of Byte-data including ID and calculated CRC8-value for validation (LSB first)
in case of an ongoing transmission or SIZE == 0 this function will return 0, otherwise 1

"DSET" -> a pointer to the dataset
*/

uint8_t DataTransmitterClass::transmitData(const dset_t DSET) {
  uint8_t commenced = 0;

  if(!bytes && DSET.SIZE) {
    bytes = DSET.SIZE;
    data = DSET.data;
    // two sync-bits + ID + CRC
    curr_byte = CRC.crcCalculation(DSET.ID, DSET.data, DSET.SIZE);
    curr_byte <<= 8;
    curr_byte += DSET.ID;
    curr_byte <<= 2;
    curr_byte += 3;

    step = 1; // each symbol requires two steps to transmit
    prev_symbol = 0; // equal to first symbol (bit) to be transmitted (which is 1)
    byte_counter = 0;
    block_size = 39;
    pin_value = 1;
  
    // start timer 2
    TCNT2  = 0; //initialize counter value to 0
    TCCR2B = prescaler;

    commenced = 1;
  }

  return commenced;
}

//################################################################################################################

/*
reads/writes an arbitrary type of data from/to the dataset
this function will return 0 in case of an error or 255 if the array goes out of scope, otherwise it will return
the next position in the data array

"X" -> a pointer to the data-element to be read/written
"SCOPE" -> the number of data elements times their size
"DSET" -> a pointer to the dataset
"POS" -> the position from which on the data element should be stored or read
"WRITE" -> flag indicating that date should be written into the array
*/

uint8_t DataTransmitterClass::dataTransfer(const void* X, const uint16_t SCOPE, const dset_t DSET, const uint8_t POS,
                                           const uint8_t WRITE) {

  uint8_t next = 0;

  if(SCOPE && (SCOPE <= DSET.SIZE-POS)) {
    for(uint16_t i=0; i<SCOPE; ++i) {
      if(WRITE) {
        *(DSET.data + POS + i) = *(((uint8_t*) X) + i);
      }
      else {
        *(((uint8_t*) X) + i) = *(DSET.data + POS + i);
      }
    }
    next = POS + SCOPE;
    if(next == DSET.SIZE) { next = 0xFF; }
  }

  return next;
}

DataTransmitterClass DT;
