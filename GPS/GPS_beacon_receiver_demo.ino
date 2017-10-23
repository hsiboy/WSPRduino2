/*
 GPS_beacon_receiver_demo
 V1.1
 Copyright (C) 2017
 Thomas Rode / DL1DUZ
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 Remote sensor data broadcast (temperature, GPS-data, Astronomical calculations)
*/

#include <DataReceiver.h>
#include <DS18B20.h>
#include <TimeLib.h>
#include <limits.h>
#include <util/atomic.h>

//##############################################################################################################
//##############################################################################################################

// definitions

// variables and constants used by the repetitive task scheduler

// a constant specifying the target loop runtime in µs
// the smaller this number the more accurate the timing
// upper limit is 16383 due to software restrictions, lower limit is set by code runtime and
// CPU-speed (around 30µS for current 16MHz AVR)
const uint16_t TLR = 2000;

// a constant specifying the loop cycles between 2 executions of the "check_data_available"-routine
// cycles = interval-time[µs]/TLR
// set to 4s
const uint32_t CHECK_DATA_AVAILABLE_LOOPS = 4000000/TLR;
// the "process datasets" loop cycle counter
uint32_t check_data_available_loop_counter = 0;

// variables and constants used by the receiver

const uint8_t BYTE_RATE = 30;
const uint8_t LOOKUP_TABLE_START = 0; // EEPROM lookup table used by CRC-library
const uint8_t RECEIVER_PIN = 2; // Arduino Nano has interrupt 1 at port D2
/*
  array of "dset_r" (defined in the DataReceiver library)
  this array holds all dataset-relevant information

  struct dset_r {
    uint8_t ID; // the ID of the dataset
    uint8_t SIZE; // number of elements
    f_ptr FUNCTION; // a function to be called for data-processing
  }
*/
dset_r dataset[2];

//##############################################################################################################
//##############################################################################################################

// dataset 0 is available -> process data
// keep processing short; no new data will be received while this one is busy!!!
void processDataset_0() {
  Serial.println(F("Received dataset with ID 0:\n"));
    
  uint8_t *data = DR.getDataArray();
    
  Serial.print(F("Latitude: "));Serial.print(*data);Serial.print(F(" deg  "));Serial.print(*(data+1));
  Serial.print(F(" min  "));Serial.print(*(data+2));Serial.print(F(" sec  "));Serial.println(char(*(data+3)));
  Serial.print(F("Longitude: "));Serial.print(*(data+4));Serial.print(F(" deg  "));Serial.print(*(data+5));
  Serial.print(F(" min  "));Serial.print(*(data+6));Serial.print(F(" sec  "));Serial.println(char(*(data+7)));

  uint16_t alt;
  DR.dataTransfer(&alt, 2, 8, 0);
  Serial.print(F("Altitude: "));Serial.print(alt);Serial.println(F(" meter"));

  Serial.print(F("Speed over ground: "));Serial.print(*(data+10));Serial.println(F(" km/h"));

  Serial.print(F("Number of satellites in use: "));Serial.println(*(data+11));

  char loc[7];
  DR.dataTransfer(loc, 7, 12, 0);
  Serial.print(F("Maidenhead locator: "));Serial.println(loc);

  int16_t temp;
  DR.dataTransfer(&temp, 2, 19, 0);
  Serial.print(F("Temperature: "));Serial.print(((float) temp)/16.0, 1);Serial.println(F(" °C"));

  time_t utc;
  DR.dataTransfer(&utc, 4, 21, 0);
  uint16_t d_s;
  // calculate time [ms] since dataset was received
  int32_t d_ms = millis() - DR.getTimestamp();
  if(d_ms > 0) { // adjust clock if delta-t is positive (no counter overflow)
    d_s = d_ms/1000 + 1;
    delay(d_s*1000 - d_ms);
    setTime(utc + d_s);
	  
    Serial.print(F("UTC: "));Serial.print(hour());Serial.print(F(":"));Serial.print(minute());Serial.print(F(":"));
    Serial.println(second());
    Serial.print(F("Date: "));Serial.print(month());Serial.print(F("/"));Serial.print(day());Serial.print(F("/"));
    Serial.println(year());
  }
    
  Serial.println(F("-------------------------------------------------"));
}

//##############################################################################################################

// dataset 1 is available -> process data
// keep processing short; no new data will be received while this one is busy!!!
void processDataset_1() {
  Serial.println(F("Received dataset with ID 1:\n"));
    
  uint8_t sog;
  DR.dataTransfer(&sog, 1, 0, 0);
  Serial.print(F("Speed over ground: "));Serial.print(sog);Serial.println(F(" km/h\n"));

  time_t rs[4];
  DR.dataTransfer(rs, 16, 1, 0);
  Serial.print(F("Sunrise: "));Serial.print(month(rs[0]));Serial.print(F("/"));Serial.print(day(rs[0]));
  Serial.print(F("   "));Serial.print(hour(rs[0]));Serial.print(F(":"));Serial.println(minute(rs[0]));
  Serial.print(F("Sunset: "));Serial.print(month(rs[1]));Serial.print(F("/"));Serial.print(day(rs[1]));
  Serial.print(F("   "));Serial.print(hour(rs[1]));Serial.print(F(":"));Serial.println(minute(rs[1]));
  Serial.print(F("Moonrise: "));Serial.print(month(rs[2]));Serial.print(F("/"));Serial.print(day(rs[2]));
  Serial.print(F("   "));Serial.print(hour(rs[2]));Serial.print(F(":"));Serial.println(minute(rs[2]));
  Serial.print(F("Moonset: "));Serial.print(month(rs[3]));Serial.print(F("/"));Serial.print(day(rs[3]));
  Serial.print(F("   "));Serial.print(hour(rs[3]));Serial.print(F(":"));Serial.println(minute(rs[3]));
  Serial.println();

  time_t pos_ts;
  DR.dataTransfer(&pos_ts, 4, 17, 0);
  Serial.print(F("Sun's position at   "));Serial.print(month(pos_ts));Serial.print(F("/"));Serial.print(day(pos_ts));
  Serial.print(F("   "));Serial.print(hour(pos_ts));Serial.print(F(":"));Serial.print(minute(pos_ts));
  Serial.print(F(":"));Serial.println(second(pos_ts));
  int32_t az;
  int16_t al;
  DR.dataTransfer(&az, 4, 21, 0);
  DR.dataTransfer(&al, 2, 25, 0);
  Serial.print(F("Azimuth in degree (N=0, E=90, S=180, W=270): "));Serial.println(((float) az)/100.0, 1);
  Serial.print(F("Altitude in degree: "));Serial.println(((float) al)/100.0, 1);
  Serial.println();

  Serial.print(F("Moon's position at   "));Serial.print(month(pos_ts));Serial.print(F("/"));Serial.print(day(pos_ts));
  Serial.print(F("   "));Serial.print(hour(pos_ts));Serial.print(F(":"));Serial.print(minute(pos_ts));
  Serial.print(F(":"));Serial.println(second(pos_ts));
  DR.dataTransfer(&az, 4, 27, 0);
  DR.dataTransfer(&al, 2, 31, 0);
  Serial.print(F("Azimuth in degree (N=0, E=90, S=180, W=270): "));Serial.println(((float) az)/100.0, 1);
  Serial.print(F("Altitude in degree: "));Serial.println(((float) al)/100.0, 1);
  int16_t phase;
  DR.dataTransfer(&phase, 2, 33, 0);
  Serial.print(F("The Moon's phase is "));Serial.print(((float) abs(phase))/100.0, 1);Serial.print(F("% "));
  if(phase > 0) {
    Serial.println(F("increasing."));
  }
  else {
    Serial.println(F("decreasing."));
  }

  Serial.println(F("-------------------------------------------------"));
}

//##############################################################################################################
//##############################################################################################################

void setup() {

/*
initializes the CRC library

Parameter:
 - the EEPROM start address to store the 256 byte CRC lookup table

 returns the next free address in EEPROM
*/
  CRC.init(LOOKUP_TABLE_START);

// create datasets
  dataset[0] = DR.createDataset(0, 25, processDataset_0);
  dataset[1] = DR.createDataset(1, 35, processDataset_1);

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
//##############################################################################################################

void loop() {
  
// variables and constants used to calculate the loop - runtime
  static uint32_t stp = micros();
  static uint32_t st;
  static int32_t dt = 0;

  const int32_t MIN_DT = LONG_MIN>>1;
  
// trigger "check data available" execution (runs every 4s)
  if(check_data_available_loop_counter == CHECK_DATA_AVAILABLE_LOOPS) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // this makes sure the following block of code will not get interrupted 
      if(DR.getStatus()&4 && DR.validateData()) {//check if data is available and consistent and if so ...
        DR.setStatus(2); // ... mark dataset as "busy reading" and ...
        (dataset + DR.getPos())->FUNCTION(); //... process it by calling its function
        DR.setStatus(0); // release dataset to enable reception of next transmission
      }
    }

    // task has been executed -> set counter to 0
    check_data_available_loop_counter = 0;
  }

// increment loop counter of all cyclical tasks
  ++check_data_available_loop_counter;

// loop delay handling
// insures that on average the target loop runtime will be met
// delays due to code execution (e.g. when processing scheduled tasks) will be taken into account
  
// check if current system time is bigger then the timestamp obtained at the last loop
  dt += TLR;
  st = micros();
  if(st > stp) { dt -= int32_t(st - stp); }
  else { dt -= int32_t((ULONG_MAX - stp) + st + 1); }
  stp = st;
  
// if runtime-budget is larger then target, insert delay-step
  if(dt > TLR) {
    delayMicroseconds(dt);
    stp += dt;
    dt = 0;
  }
  else {
// avoid overflow, in case the budget is constantly negative (loop is too slow to meet target loop time)
    if(dt < MIN_DT) { dt = 0; }
  }

}