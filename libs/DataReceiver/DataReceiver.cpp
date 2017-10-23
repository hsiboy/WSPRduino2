/*
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
*/

//################################################################################################################
//includes
//################################################################################################################

#include <DataReceiver.h>

//################################################################################################################
//declarations
//################################################################################################################

DataReceiverClass *DataReceiverClass::this_instance;

//################################################################################################################
//global functions
//##############################################################################################################

void DataReceiverClass::isr() {
  this_instance -> DRisr();
}

//################################################################################################################
//private functions
//################################################################################################################

//interrupt service routine

void DataReceiverClass::DRisr() {
  // Variable holding the microsecond-timestamp, when the interrupt service routine was last called.
  static uint32_t msl = 0;
  // A dummy to hold the current microsecond-timestamp.
  static uint32_t msc;
  // Variable holding the time in microseconds passed since the last call of the interrupt service routine.
  static uint32_t dt;

// Catch the current microsecond-timestamp when entering the interrupt service routine.
  msc = micros();
// calculate the time in μs passed since the last call of this routine.
  dt = msc - msl;
// Update the variable holding the timestamp of the last call of this routine.
  msl = msc;

// check dt against calculated time step thresholds
  if(dt<T1 || dt>T3) {
    resetReception();
  }
  else {
    if(dt>T2 || !skip_next_short) {
      skip_next_short = 1;// set "skip next short pulse"-flag

      if(int_counter) {//preamble was ok, data is being received
	if(dt>T2) { // current bit != previous bit
	  prev_bit = !prev_bit;
	}
	if(prev_bit) {//set bit
	  byte_value += bit_value;
	}

   	if(bit_value & 0x80) { // last bit has been reached -> switch to next byte
	  if(!byte_counter) {// this is the first Byte (ID-Byte)
	    if(!status) {//check if data array is available
	      uint8_t id_exists = 0;
	      for(uint8_t i=0; i<dataset_count; ++i) {
	        if((dataset+i)->ID==byte_value) {//... check if ID exists in this system and ...
	          // ... if so, flag it as busy and record timestamp
	          pos = i;
	          status = 1;
	          timestamp = millis() - Tl;
		  storeData();
		  id_exists = 1;
		  break;
		}
	      }
	      if(!id_exists) { resetReception(); } //if ID could not be found, reset reception
	    }
	    else {
	      resetReception();
	    }
	  }
	  else {
	    storeData();
            //terminate reception and set status to "data available" if last element has been stored
	    if(byte_counter == (dataset+pos)->SIZE + 2) {
	      status = 4;
	      resetReception();
	    }
	  }
	}
	else {
	  bit_value <<= 1; // bit_value *= 2
	}
      }
      else {//preamble is being received
	if(dt>T2) {//peramble-syntax has been violated -> cancel reception
	  resetReception();
	}
	else {
	  ++int_counter;
	}
      }
    }
    else {
      skip_next_short = 0;
    }
  }
}

//################################################################################################################

// support function for the ISR

void DataReceiverClass::resetReception() {
  int_counter = 0;
  byte_counter = 0;
  bit_value = 1;
  byte_value = 0;
  prev_bit = 1;
  skip_next_short = 1;

  if(status & 1) {//check if dataset is being written and if so, reset flag
    status = 0;
  }
}

// ######################################################################################################################

// support function for the ISR

void DataReceiverClass::storeData() {
  *(data_array + byte_counter) = byte_value; // transfer received byte into array
  bit_value = 1; // reset bit-value to first bit
  byte_value = 0; // reset byte-value
  ++byte_counter; // increment byte-counter
}

//################################################################################################################
//public functions
//################################################################################################################

/*
initializes the library
returns 1 if successfurl, otherwise 0

"BYTE_RATE" -> the target gross data transfer rate in Byte/s set at the transmitter. Valid entries are 4...255.
"RECEIVER_PIN" -> the digital, interrupt-capable pin the receiver is connected to
"dsets" -> array of "dset_r", holding all parameter relevant for the datasets
"DATASET_COUNT" -> the number of datasets
*/

uint8_t DataReceiverClass::init(const uint8_t BYTE_RATE, const uint8_t RECEIVER_PIN, dset_r* dsets,
                                const uint8_t DATASET_COUNT) {
  if(!is_initialized && BYTE_RATE>3 && DATASET_COUNT) {
    receiver_input = RECEIVER_PIN;
    pinMode(receiver_input, INPUT);

    dataset = dsets;
    dataset_count = DATASET_COUNT;

    // allocate memory matching the size of the biggest dataset
    array_size = 0;
    for(uint8_t i=0; i<dataset_count; ++i) {
      array_size = max(array_size, (dataset+i)->SIZE);
    }
    data_array = (uint8_t*) malloc(array_size + 2);
    if(data_array != NULL) {
    
      // calculate time step threshold [μs] for given Byte-rate
      T1 = 31250/BYTE_RATE;
      T2 = 3*T1;
      T3 = 5*T1;
      Tl = 1375/BYTE_RATE; // 44*T1

      resetReception(); // just to initialize the variables

      this_instance = this;
      attachInterrupt(digitalPinToInterrupt(receiver_input), isr, CHANGE);

      is_initialized = 1;
    }
  }

  return is_initialized;
}

//################################################################################################################

// enables/disables the receiver by turning the interrupt on/off

void DataReceiverClass::enableReceiverInput(const uint8_t ENABLE) {
  if(is_initialized) {
    if(ENABLE) {
      attachInterrupt(digitalPinToInterrupt(receiver_input), isr, CHANGE);
    }
    else {
      detachInterrupt(digitalPinToInterrupt(receiver_input));
    }
  }
}

//################################################################################################################

/*
creates a structure of the type dset_r (receiver dataset parameter)

"ID" -> the ID of the dataset (0...255)
"SIZE" -> the size of the dataset (1...255 Byte)
"FUNCTION" -> a function to be called for data-processing
*/

dset_r DataReceiverClass::createDataset(const uint8_t ID, const uint8_t SIZE, const f_ptr FUNCTION) {
  dset_r dummy;

  dummy.ID = ID; //the ID of the dataset
  dummy.SIZE = SIZE; // number of elements
  dummy.FUNCTION = FUNCTION; // a function to be called for data-processing

  return dummy;
}

//################################################################################################################

/*
returns the status of the data array (idle = 0; busy writing = 1; busy reading = 2; data available = 4)
*/
uint8_t DataReceiverClass::getStatus() {
  return status;
}

//################################################################################################################

/*
sets the status of the data array (idle = 0; busy writing = 1; busy reading = 2; data available = 4)
*/
void DataReceiverClass::setStatus(uint8_t stat) {
  status = stat;
}

//################################################################################################################

/*
returns the timestamp [ms] of the data array (time when reception started, latencies accounted for)
*/
uint32_t DataReceiverClass::getTimestamp() {
  return timestamp;
}

//################################################################################################################

/*
returns the ID-value currently stored in the data-array (the ID of the dataset last received)
*/
uint8_t DataReceiverClass::getID() {
  return *data_array;
}

//################################################################################################################

/*
returns the element-id of the dataset corresponding to the ID-value currently stored in the array
i.e. if the ID received corresponds to dataset[3] then this function would return 3
*/
uint8_t DataReceiverClass::getPos() {
  return pos;
}

//################################################################################################################

/*
returns the CRC-value currently stored in the array
*/
uint8_t DataReceiverClass::getCRC() {
  return *(data_array + 1);
}

//################################################################################################################

/*
returns a pointer to the data section of the data array
*/
uint8_t* DataReceiverClass::getDataArray() {
  return (data_array + 2);
}

//################################################################################################################

// calculates the CRC for the dataset currently in the buffer and compares it to the one received
// returns 1 if matching, otherwise returns 0

uint8_t DataReceiverClass::validateData() {
  uint8_t is_valid = 0;
  if(*(data_array+1) == CRC.crcCalculation((dataset+pos)->ID, data_array+2, (dataset+pos)->SIZE)) {
    is_valid = 1;
  }

  return is_valid;
}

//################################################################################################################

/*
reads/writes an arbitrary type of data from/to the data array
this function will return 0 in case of an error or 255 if the array goes out of scope, otherwise it will return
the next position in the data array

"X" -> a pointer to the data-element to be read/wrote
"SCOPE" -> the number of data elements times their size
"POS" -> the position from which on the data element should be stored or read
"WRITE" -> flag indicating that date should be written into the array
*/

uint8_t DataReceiverClass::dataTransfer(const void* X, const uint16_t SCOPE, const uint8_t POS, const uint8_t WRITE) {

  uint8_t next = 0;

  if(SCOPE && (SCOPE <= array_size-POS)) {
    for(uint16_t i=0; i<SCOPE; ++i) {
      if(WRITE) {
        *(data_array + POS + i + 2) = *(((uint8_t*) X) + i);
      }
      else {
        *(((uint8_t*) X) + i) = *(data_array + POS + i + 2);
      }
    }
    next = POS + SCOPE;
    if(next == array_size) { next = 0xFF; }
  }

  return next;
}

DataReceiverClass DR;
