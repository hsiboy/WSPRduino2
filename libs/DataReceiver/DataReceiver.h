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

#ifndef DataReceiver_h_
#define DataReceiver_h_

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

typedef void (*f_ptr)(); // funktion pointer

// structure containing all relevant parameter of a dataset
struct dset_r {
  uint8_t ID; // the ID of the dataset
  uint8_t SIZE; // number of elements
  f_ptr FUNCTION; // a function to be called for data-processing
};

//################################################################################################################

class DataReceiverClass {

public:

  uint8_t init(const uint8_t BYTE_RATE, const uint8_t RECEIVER_PIN, dset_r* d_set,
            const uint8_t DATASET_COUNT);

  void enableReceiverInput(const uint8_t ENABLE);
  dset_r createDataset(const uint8_t ID, const uint8_t SIZE, const f_ptr FUNCTION);
  void setStatus(uint8_t stat);
  uint8_t getStatus();
  uint32_t getTimestamp();
  uint8_t getID();
  uint8_t getPos();
  uint8_t getCRC();
  uint8_t validateData();
  uint8_t dataTransfer(const void* X, const uint16_t SCOPE, const uint8_t POS, const uint8_t WRITE);
  uint8_t* getDataArray();

private:

  // Flag holding the init-state of this routine
  uint8_t is_initialized = 0;
  // holding the receiver input pin
  uint8_t receiver_input = 0;
  // time step thresholds [μs] for given Byte-rate
  uint16_t T1, T2, T3;
  // latency between beginning of transmission and recognition of ID
  uint16_t Tl;
  // for synchronisation purposes
  uint8_t int_counter;
  // counting the bytes already receiver
  uint16_t byte_counter;
  // for internal use (bitwise manipulation of the byte being received)
  uint8_t bit_value;
  // holds the value of the byte currently being received
  uint8_t byte_value;
  // value of the previous bit received
  uint8_t prev_bit;
  // the position of the dataset corresponding to the ID-value currently stored in the array
  uint8_t pos;
  // flag controlling the datastream processing (used at ISR)
  uint8_t skip_next_short;

// pointer to dset_r-array
  dset_r* dataset;
// the number of datasets (size of "dataset")
  uint8_t dataset_count;
// pointer to an array containing the received data
  uint8_t* data_array;
// the size of the data array
  uint8_t array_size = 0;
// the status of the data array (idle = 0; busy writing = 1; busy reading = 2; data available = 4)
  volatile uint8_t status = 0; 
// the timestamp [ms] when the data was received
  uint32_t timestamp;

  static DataReceiverClass *this_instance;
  static void isr();
  void DRisr();
  void resetReception();
  void storeData();

};

extern DataReceiverClass DR;

#endif // DataReceiver_h_
