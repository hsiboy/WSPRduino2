"DataTransmitter"
Library supporting wireless or wire-bound serial data broadcasting
(e.g. sensor data via a 2-wire connection or radio broadcast on an ISM-frequency)
The transmission is DC-free (Manchester-coded) and error-checked (CRC8)
Requires library "CRCGenerator"

V1.1 / Copyright (C) 2017, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Required ressources:
 - 1 digital output pin
 - Timer 2 (conflicting with the functions  "analogWrite()" at pins 3 & 11 and "tone()" (all pins)).

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Datatypes:

// structure representing a transmitter-dataset
struct dset_t {
  uint8_t* data; // a pointer to the actual data array
  uint8_t ID; // the ID of the dataset
  uint8_t SIZE; // number of elements
};

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Methods:

void init(const uint8_t BYTE_RATE, const uint8_t TRANSMITTER_PIN, const uint8_t IS_LOW_ACTIVE)

initializes the library

"BYTE_RATE" -> the target gross data transfer rate in Byte/s (the algorithm will attempt getting as
 close as possible to this number). Valid entries are 4...255. A small overhead is introduced by the
 system (2 Byte for CRC & ID + 3.5 bit for the preamble)
"TRANSMITTER_PIN" -> the digital pin the transmitter is connected to
"IS_LOW_ACTIVE" -> if True, the transmitter-logic is low-active (low->TX on)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

dset_t createDataset(const void* X, const uint8_t ID, const uint8_t SCOPE)

creates a dataset of the type dset_t (transmission data)

"X" -> a pointer to the data-element (single variable, array, ...)
"ID" -> the ID of the dataset (0...255)
"SCOPE" -> the number of data elements times their size (1...255 Byte)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t transmitData(const dset_t DSET)

transmits a set of Byte-data including ID and calculated CRC8-value for validation (LSB first)
in case of an ongoing transmission or SIZE == 0 this function will return 0, otherwise 1

"DSET" -> a pointer to the dataset

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t dataTransfer(const void* X, const uint16_t SCOPE, const dset_t DSET, const uint8_t POS,
                     const uint8_t WRITE)

reads/writes an arbitrary type of data from/to the dataset
this function will return 0 in case of an error or 255 if the array goes out of scope, otherwise it
will return the next position in the data array

"X" -> a pointer to the data-element to be read/written
"SCOPE" -> the number of data elements times their size
"DSET" -> the dataset
"POS" -> the position from which on the data element should be stored or read
"WRITE" -> flag indicating that date should be written into the array

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Simple Arduino example code: Cyclical broadcasting of some data / not particularly meaningful but
just a little excercise to show how the library can be used
*/

#include <DataTransmitter.h>

// some data to transmit
const float Pi = 3.14159; // first element to be transmitted
unsigned long system_time; // second element to be transmitted
char test_string[29] = "This is a test transmission."; // third element to be transmitted

// variables and constants used by the transmitter
const byte TRANSMITTER_PIN = 12; // the data-port
const byte BYTE_RATE = 30; // the byte rate 4...255 Bytes/s
const byte LOOKUP_TABLE_START = 0; // start address of the 256 byte CRC-lookup table in EEPROM
const byte LED = 13; // the onboard LED

dset_t dataset[2]; // two dataset-structures, one for each transmission sequence
byte dset[8]; // an array of 8 byte to merge the 4-byte elements "Pi" and "system_time" into one
              // dataset-structure (one transmission sequence)

//###################################################################################################

void setup() {

  pinMode(LED, OUTPUT); // activate the onboard LED
  
/*
initializes the CRC8 library

Parameter:
 - the EEPROM start address to store the 256 byte CRC lookup table

 returns the next free address in EEPROM
*/
  CRC.init(LOOKUP_TABLE_START);
  
/*
initializes the DataTransmitter library

Parameter (f.l.t.r.):
 - the target gross data transfer rate in Byte/s (the algorithm will attempt getting as close as
   possible to this number. Minimum value is 4 Byte/s.
 - the digital pin the transmitter is connected to
 - "IS_LOW_ACTIVE" -> if True, the transmitter-logic is low-active (low->TX on)
*/
  DT.init(BYTE_RATE, TRANSMITTER_PIN, 0);

// "dset" is assigned to dataset 0 which gets the ID 10 (randomly choosen)
  dataset[0] = DT.createDataset(dset, 10, sizeof(dset));
 // "test_string" is assigned to dataset 1 which gets the ID 11  (randomly choosen)
  dataset[1] = DT.createDataset(test_string, 11, sizeof(test_string));
  
// copy "Pi" into "dataset[0]" beginning at Byte 0
  DT.dataTransfer(&Pi, sizeof(Pi), dataset[0], 0, 1);

}

//###################################################################################################

void loop() {

// record system time        
  system_time = millis();
// copy "system_time" into "dataset[0]" beginning at Byte 4
  DT.dataTransfer(&system_time, sizeof(system_time), dataset[0], 4, 1);
  DT.transmitData(dataset[0]); // broadcast dataset 0 (containing "Pi" and "system_time")
  
// pause 5s (onboard LED on for 0.5s to signal transmission)
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(4500);
  
  DT.transmitData(dataset[1]); // broadcast dataset 1 (containing "test_string")

// pause 5s (onboard LED on for 0.5s to signal transmission)
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(4500);

}