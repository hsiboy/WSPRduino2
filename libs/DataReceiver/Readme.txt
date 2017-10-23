"DataReceiver"
Library supporting wireless or wire-bound serial data broadcasting
(e.g. sensor data via a 2-wire connection or radio broadcast on an ISM-frequency)
The transmission is DC-free (Manchester-coded) and error-checked (CRC8)
Requires library "CRCGenerator"
  
V1.1 / Copyright (C) 2017, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Required ressources:
 - 1 digital, interrupt-capable input pin

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Datatypes:

typedef void (*f_ptr)(); // funktion pointer

// structure containing all relevant parameter of a dataset
struct dset_r {
  uint8_t ID; // the ID of the dataset
  uint8_t SIZE; // number of elements
  f_ptr FUNCTION; // a function to be called for data-processing
};

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Methods:

void init(const uint8_t BYTE_RATE, const uint8_t RECEIVER_PIN, const dset* DATASET,
          const uint8_t NUMBER_OF_DATASETS)

initializes the library
returns 1 if successfurl, otherwise 0

"BYTE_RATE" -> the target gross data transfer rate in Byte/s set at the transmitter. Valid entries are 4...255.
"RECEIVER_PIN" -> the digital, interrupt-capable pin the receiver is connected to
"dsets" -> array of "dset_r", holding all parameter relevant for the datasets
"DATASET_COUNT" -> the number of datasets

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void enableReceiverInput(const uint8_t ENABLE)

enables/disables the receiver by turning the interrupt on/off

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

dset_r createDataset(const uint8_t ID, const uint8_t SIZE, const f_ptr FUNCTION)

creates a structure of the type dset_r (receiver dataset parameter)

"ID" -> the ID of the dataset (0...255)
"SIZE" -> the size of the dataset (1...255 Byte)
"FUNCTION" -> a function to be called for data-processing

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t dataTransfer(const void* X, const uint16_t SCOPE, const uint8_t POS, const uint8_t WRITE)

reads/writes an arbitrary type of data from/to the data array
this function will return 0 in case of an error or 255 if the array goes out of scope, otherwise it will return
the next position in the data array

"X" -> a pointer to the data-element to be read/wrote
"SCOPE" -> the number of data elements times their size
"POS" -> the position from which on the data element should be stored or read
"WRITE" -> flag indicating that date should be written into the array

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setStatus(uint8_t stat)

sets the status of the data array (idle = 0; busy writing = 1; busy reading = 2; data available = 4)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t getStatus()

returns the status of the data array (idle = 0; busy writing = 1; busy reading = 2; data available = 4)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint32_t getTimestamp() {
  return timestamp;
}

returns the timestamp [ms] of the data array (time when reception started, latencies accounted for)

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t getID()

returns the ID of the currently stored data

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t getPos()

returns the element-id of the dataset corresponding to the ID-value currently stored in the array
i.e. if the ID received corresponds to dataset[3] then this function would return 3

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t getCRC()

returns the CRC-value of the currently stored data

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t* getDataArray()

returns a pointer to the data section of the data array

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t validateData();

calculates the CRC for the dataset currently in the buffer and compares it to the one received
returns 1 if matching, otherwise returns 0

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Simple Arduino example code: Cyclical broadcasting of some data / not particularly meaningful but
just a little excercise to show how the library can be used
*/

#include <DataReceiver.h>
#include <util/atomic.h>

//##############################################################################################################

// definitions

// variables and constants used by the receiver

const uint8_t BYTE_RATE = 30;
const uint8_t LOOKUP_TABLE_START = 0; // EEPROM lookup table used by CRC-library
const uint8_t RECEIVER_PIN = 2; // Arduino Nano has interrupt 1 at port D2
/*
  array of "dset_r" (defined in the DataReceiver library)
  this array holds all dataset-relevant parameter

  struct dset_r {
    uint8_t ID; // the ID of the dataset
    uint8_t SIZE; // number of elements
    f_ptr FUNCTION; // a function to be called for data-processing
  }
*/
dset_r dataset[2];

//##############################################################################################################

// dataset 0 is available -> process data
// keep processing short; no new data will be received while this one is busy!!!
void processDataset_0() {
  Serial.print(F("It was received at system time: "));Serial.println(DR.getTimestamp());
  // read the data
  float Pi;
  uint32_t sys_time;
  DR.dataTransfer(&Pi, 4, 0, 0);
  DR.dataTransfer(&sys_time, 4, 4, 0);
  // print the data
  Serial.print(F("The dataset contains PI which is "));Serial.print(Pi, 5);
  Serial.print(F(" as well as the transmitters system time which is "));Serial.println(sys_time);
  Serial.println(F("------------------------------------------------------------------------------------------------"));
}

//##############################################################################################################

// dataset 1 is available -> process data
// keep processing short; no new data will be received while this one is busy!!!
void processDataset_1() {
  Serial.print(F("It was received at system time: "));Serial.println(DR.getTimestamp());
  // read the data
  char test_string[29];
  DR.dataTransfer(test_string, sizeof(test_string), 0, 0);
  // print the data
  Serial.print(F("The dataset contains a string which says: "));Serial.println(test_string);
  Serial.println(F("------------------------------------------------------------------------------------------------"));
}

//##############################################################################################################

void setup() {

/*
initializes the CRC library

Parameter:
 - the EEPROM start address to store the 256 byte CRC lookup table

 returns the next free address in EEPROM
*/
  CRC.init(LOOKUP_TABLE_START);

/* 
create datasets
dataset 0 has the ID 10, a size of 8 byte and is linked to the function "processDataset_0"
dataset 1 has the ID 11, a size of 29 byte and is linked to the function "processDataset_1"
*/
  dataset[0] = DR.createDataset(10, 8, processDataset_0);
  dataset[1] = DR.createDataset(11, 29, processDataset_1);

/*
initializes the DataReceiver

Parameter DataReceiver (f.l.t.r.):
"BYTE_RATE" -> the data transfer rate set at the transmitter in Byte/s
"RECEIVER_PIN" -> the digital, interrupt-capable pin the receiver is connected to
"d_set" -> array of "dset", holding all dataset-relevant information
"DATASET_COUNT" -> the number of datasets (size of DATASET)
*/
  if(!DR.init(BYTE_RATE, RECEIVER_PIN, dataset, 2)) {
    pinMode(13, OUTPUT); // onboard LED will start flashing if receiver could not be initialized
    while(true) {
      digitalWrite(13, !digitalRead(13));
      delay(200);
      digitalWrite(13, !digitalRead(13));
      delay(200);
    }
  }

  Serial.begin(9600);
  Serial.println(F("Communication to PC has been established.\n"));
  
}

//##############################################################################################################

void loop() {
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // this makes sure the following block of code will not get interrupted 
    if(DR.getStatus()&4 && DR.validateData()) {//check if data is available and consistent and if so ...
      DR.setStatus(2); // ... mark dataset as "busy reading" and ...
      Serial.print(F("Received a set of data with ID "));Serial.println(DR.getID());//... output its ID and ...
      (dataset + DR.getPos())->FUNCTION(); //... process it by calling its function
      DR.setStatus(0); // release dataset to enable reception of next transmission
    }
  }

  delay(500); // repeat loop every 500ms

}
