/*
 GPS_beacon
 V1.1
 Copyright (C) 2017
 Thomas Rode / DL1DUZ
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 Remote sensor data broadcast (temperature, GPS-data, Astronomical calculations)
*/

#include <limits.h>
#include <DataTransmitter.h>
#include <DS18B20.h>
#include <SoftwareSerial.h>
#include <MoonSun.h>

// ######################################################################################################################

// variables used by the repetitive task scheduler

// a constant specifying the target loop runtime in µs
// the smaller this number the more accurate the timing
// upper limit is 16383 due to software restrictions, lower limit is set by code runtime and
// CPU-speed (around 30µS for current 16MHz AVR)
const uint16_t TLR = 15000;

// a constant specifying the loop cycles between 2 executions of the "transmit data"-routine
// cycles = interval-time[µs]/TLR
// set to 19s
const uint16_t TRANSMIT_DATA_LOOPS = 19000000/TLR;
// the "transmit data" loop cycle counter
uint16_t transmit_data_loop_counter = 0;

// variables and constants used by the status-LED

const uint8_t STATUS_LED_PIN = 13;

// variables and constants used by the temperature sensor

const uint8_t TEMP_SENSOR_PIN = 11;
const uint8_t RESOLUTION = 12; // 12 bit resolution == 0.0625°C

// variables and constants used by the transmitter

/* an array to hold the data to be transmitted
for the dataset with ID 0 it contains 25 Byte:

    GPS latitude in degree / uint8_t
    GPS latitude in minutes / uint8_t
    GPS latitude in seconds / uint8_t
    GPS latitude orientation ('N', 'n', 'S', 's') / uint8_t
    
    GPS longitude in degree / uint8_t
    GPS longitude in minutes / uint8_t
    GPS longitude in seconds / uint8_t
    GPS longitude orientation ('N', 'n', 'S', 's') / uint8_t
    
    GPS altitude in meter / uint16_t
    
    GPS speed over ground in km/h / uint8_t
    
    GPS number of satellites used / uint8_t

    6-character Maidenhead locator / char[7] (7 Byte)
    
    outside temperature * 16 / int16_t
    
    the current UTC / time_t (4 Byte)

for the dataset with ID 1 it contains 35 Byte:
    GPS speed over ground in km/h / uint8_t

    the Moon's and Sun's rise and set time / time_t[4] (Sunrise, Sunset, Moonrise, Moonset) / (16 Byte)

    the timestamp for which the following data was calculated / time_t (4 Byte)
    the Sun's azimuth in degree*100 (North = 0, East = 90, South = 180, West = 270) / int32_t
    the Sun's altitude in degree*100 / int16_t
    the Moon's azimuth in degree*100 (North = 0, East = 90, South = 180, West = 270) / int32_t
    the Moon's altitude in degree*100 / int16_t
    the Moon's phase in %*100 (positive for increasing phase) / int16_t
*/

uint8_t data[35];

// defines the dataset which is to be transmitted next
uint8_t current_dataset = 1;

const uint8_t TRANSMITTER_PIN = 12;
const uint8_t BYTE_RATE = 30;
const uint8_t LOOKUP_TABLE_START = 0;

// $GPZDA,184020.00,31,07,2017,00,00*68
// $GPGGA,184212.00,5104.21487,N,01339.75269,E,1,09,0.94,122.8,M,43.7,M,,*52
// $GPVTG,77.52,T,,M,0.004,N,0.008,K,A*06

// variables and constants used by the GPS (NEO6MV2)

const uint8_t gpsRX_PIN = 10;
const uint8_t gpsTX_PIN = 9;

uint8_t buffer[82] = {0};

SoftwareSerial gpsSerial(gpsRX_PIN, gpsTX_PIN);

uint8_t lat_deg, lat_min, lat_sec, long_deg, long_min, long_sec; // latitude and longtitude in degree, minutes, seconds
uint8_t lat_o, long_o; // latitude and longtitude orientation (E,W,N,S)
int16_t alt; // altitude in Meter
uint8_t sat; // number of sat's locked
uint8_t sog; // speed over ground [km/h]

// variables and constants used for astronomical calculations

time_t t = 0;

// functions

// ######################################################################################################################

void LatLongToLocator(char* locator, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o, uint8_t long_deg,
                      uint8_t long_min, uint8_t long_sec, uint8_t long_o) {

  uint32_t step = 648000;

// convert position in degree, min, sec to sec and change reference to 180W/90S
// 1degree == 3600s   1min == 60s
  uint32_t llo = ((uint32_t) long_deg)*3600 + ((uint32_t) long_min)*60 + long_sec;
  if(long_o=='W' || long_o=='w') {
    llo = -llo;
  }
  llo += step;

  step >>= 1;

  uint32_t lla = ((uint32_t) lat_deg)*3600 + ((uint32_t) lat_min)*60 + lat_sec;
  if(lat_o=='S' || lat_o=='s') {
    lla = -lla;
  }
  lla += step;

  uint8_t base;
  step = 864000;

  for(uint8_t i=0; i<6; i+=2) {
    if(i&2) {
      base = '0';
      step /= 5;
    }
    else {
      base = 'A';
      step /= 12;
    }

    *(locator + i) = base + llo/step;
    llo %= step;
    step >>= 1;

    *(locator + i + 1) = base + lla/step;
    lla %= step;
  }
  *(locator + 6) ='\0';

}

// ######################################################################################################################
// flashes the LED x times

void flashLED(uint8_t pulses, uint8_t interval) {
  for(uint8_t i=0; i<pulses; ++i) {
    digitalWrite(STATUS_LED_PIN, 1);
    delay(interval);
    digitalWrite(STATUS_LED_PIN, 0);
    delay(interval);
  }
}

// ######################################################################################################################
// finds the position of the x'th occurance of ',' in the buffer string and returns this value + 1
// returns 0, if given element does not exist

uint8_t findSeparator(uint8_t element) {
  uint8_t pos = 0;
  for(uint8_t i=0; i<sizeof(buffer); ++i) {
    if(buffer[i]==',') {
      --element;
    }
    if(!element) {
      pos = i + 1;
      break;
    }
  }
  
  return pos;
}

// ######################################################################################################################
// converting a character representing a positive (hexa)decimal number to its numerical value

uint8_t hexCharToByte(uint8_t c) {
  if (c >= 'a') {
    c = c - 'a' + 10;
  }
  else {
    if (c >= 'A') {
      c = c - 'A' + 10;
    }
    else {
      c = c - '0';
    }
  }

  return c;
}

// ######################################################################################################################
// converts a sequence of 1...3 successive characters in the buffer representing a positive decimal number to
// its numerical value (max 255)

uint8_t decStringToByte(uint8_t pos, uint8_t length) {
  uint8_t mult = 1;
  uint8_t result = 0;
  for(int8_t i=length-1; i>=0; --i) {
    result += hexCharToByte(buffer[pos+i])*mult;
    mult *= 10;
  }
  
  return result;
}

// ######################################################################################################################
// converts a set of characters in the buffer representing the decimal-places of an floating point number
// to its numerical value

float floatStringToFloat(uint8_t b, uint8_t e) {
  int32_t result = 0;
  uint8_t dp = 0;
  uint32_t mult = 1;
  int8_t i;
  
  for(i=e; i>=b; --i) {

    switch(buffer[i]) {
      case '.':
        dp = i;
      break;
      case '-':
        result *= -1;
      break;
      default:
        result += hexCharToByte(buffer[i])*mult;
        mult *= 10;
      break;
    }
  }

  mult = 1;
  for(i=0; i<(e-dp); ++i) {
    mult *= 10;
  }
  return float(result)/mult;
}

// ######################################################################################################################
// converts a set of characters in the buffer representing the decimal-places of an floating point number
// to its numerical value (integer part only)

int16_t floatStringToInteger(uint8_t b, uint8_t e) {
  int16_t result = 0;
  uint8_t dp_found = 0;
  uint16_t mult = 1;
  
  for(int8_t i=e; i>=b; --i) {
    switch(buffer[i]) {
      case '.':
        dp_found = 1;
      break;
      case '-':
        result *= -1;
      break;
      default:
        if(dp_found) {
          result += hexCharToByte(buffer[i])*mult;
          mult *= 10;
        }
      break;
    }
  }

  return result;
}

// ######################################################################################################################
// clears the gpsSerial input buffer

void clearInputBuffer() {
  while(gpsSerial.available()) {
    gpsSerial.read();
    delayMicroseconds(1000);
  }
}

// ######################################################################################################################
// attempts receiving a dataset from the GPS (timeout = 1.5s)
// returns 1 if successful, otherwise 0

uint8_t receiveDataset() {
  uint8_t success = 0;

  for(uint16_t i=0; i<1500; ++i) {
    if(gpsSerial.available()) {
      gpsSerial.readBytesUntil('\n', buffer, sizeof(buffer));

// calculate CRC
      uint8_t crc_calc = 0, crc_sent;
      for(uint8_t i=1; i<sizeof(buffer); ++i) {
        if(buffer[i]!='*') {
          crc_calc^=buffer[i];
        }
        else {
          crc_sent = (hexCharToByte(buffer[i+1])<<4) + hexCharToByte(buffer[i+2]);
          break;
        }
      }
// check calculated CRC vs. sent one
      if(crc_calc==crc_sent) {
        success = 1;
      }
      break;
    }
// pause 1ms before next loop  
    delayMicroseconds(1000);
  }

  return success;
}

// ######################################################################################################################
// aquires the speed over ground in km/h
// $GPVTG,77.52,T,,M,0.004,N,0.008,K,A*06

uint8_t getSpeedOverGround() {
// poll VTG dataset
  clearInputBuffer();
  gpsSerial.println(F("$EIGPQ,VTG*23"));
// if data has been received and speed is available
  uint8_t result = 0;
  if(receiveDataset() && buffer[findSeparator(7)]!=',') {
    result = floatStringToInteger(findSeparator(7), findSeparator(8)-2);
  }

  return result;
}

// ######################################################################################################################
void setup() {

  pinMode(STATUS_LED_PIN, OUTPUT);  

/*
initializes the temperature sensor

Parameter:
 - the pin the sensor is connected to
 - the required sensor resolution (9...12 bit corrseponding to 0.5°C ... 0.0625°C)
*/
  if(!TS.init(TEMP_SENSOR_PIN, RESOLUTION)) {
    while(true) {// if initialisation has failed, status-LED will be at 4Hz
      digitalWrite(STATUS_LED_PIN, 1);
      delay(125);
      digitalWrite(STATUS_LED_PIN, 0);
      delay(125);
    }
  }

/*
initializes the GPS
*/
// set baudrate
  gpsSerial.begin(9600);
  delay(1000);
  gpsSerial.println(F("$PUBX,41,1,7,3,38400,0*20"));
  gpsSerial.end();
  delay(1000);
  gpsSerial.begin(38400);
  delay(1000);
  gpsSerial.setTimeout(500);
  
// configure cyclical outputs (all off)
  gpsSerial.println(F("$PUBX,40,GGA,0,0,0,0,0,0*5A"));
  delay(100);
  gpsSerial.println(F("$PUBX,40,GSA,0,0,0,0,0,0*4E"));
  delay(100);
  gpsSerial.println(F("$PUBX,40,GSV,0,0,0,0,0,0*59"));
  delay(100);
  gpsSerial.println(F("$PUBX,40,GLL,0,0,0,0,0,0*5C"));
  delay(100);
  gpsSerial.println(F("$PUBX,40,RMC,0,0,0,0,0,0*47"));
  delay(100);
  gpsSerial.println(F("$PUBX,40,VTG,0,0,0,0,0,0*5E"));
  delay(100);
  gpsSerial.println(F("$PUBX,40,ZDA,0,0,0,0,0,0*44"));
  delay(100);

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
 - the target gross data transfer rate in Byte/s (the algorithm will attempt getting as close as possible to this
   number. Minimum value is 4 Byte/s.
 - the digital pin the transmitter is connected to
 - "IS_LOW_ACTIVE" -> if True, the transmitter-logic is low-active (low->TX on)
*/
  DT.init(BYTE_RATE, TRANSMITTER_PIN, 0);

  flashLED(20, 62);
}

// ######################################################################################################################
void loop() {

// variables and constants used to calculate the loop - runtime
  static uint32_t stp = micros();
  static uint32_t st;
  static int32_t dt = 0;

  const int32_t MIN_DT = LONG_MIN/2;

// trigger "transmit data" execution (runs every 19s)
  if(transmit_data_loop_counter == TRANSMIT_DATA_LOOPS) {
    
    uint8_t base, pos; // used by the parsing process
    
    if(current_dataset) {
      
// poll GGA dataset
      clearInputBuffer();
      gpsSerial.println(F("$EIGPQ,GGA*27"));
// if data has been received and position fix is ok
      if(receiveDataset() && buffer[findSeparator(2)]!=',') {
// parse position-string
        base = findSeparator(2);
        lat_deg = decStringToByte(base, 2);
        base += 2;
        lat_min = decStringToByte(base, 2);
        base += 2;
        lat_sec = floatStringToFloat(base, findSeparator(3)-2)*60;
    
        base = findSeparator(4);
        long_deg = decStringToByte(base, 3);
        base += 3;
        long_min = decStringToByte(base, 2);
        base += 2;
        long_sec = floatStringToFloat(base, findSeparator(5)-2)*60;

        lat_o = buffer[findSeparator(3)];
        long_o = buffer[findSeparator(5)];

        alt = floatStringToInteger(findSeparator(9), findSeparator(10)-2);

        sat = decStringToByte(findSeparator(7), 2);

        sog = getSpeedOverGround();

// convert coordinates to 6-character Maidenhead-locator
        char locator[7];
        LatLongToLocator(locator, lat_deg, lat_min, lat_sec, lat_o, long_deg, long_min, long_sec, long_o);

// attempt reading temp-sensor (temp = 0x8000==-32768==0b1000 0000 0000 0000 in case of error)
        int16_t temp = 0x8000;
        if(TS.convertTemp(1)) {
          temp = TS.getTemp();
        }

// Create a dataset (pointer to data, id, size)
        dset_t dataset = DT.createDataset(data, 0, 25);

// transfer data into array (part 1)
        pos = 0;
        pos = DT.dataTransfer(&lat_deg, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&lat_min, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&lat_sec, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&lat_o, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&long_deg, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&long_min, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&long_sec, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&long_o, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&alt, 2, dataset, pos, 1);
        pos = DT.dataTransfer(&sog, 1, dataset, pos, 1);
        pos = DT.dataTransfer(&sat, 1, dataset, pos, 1);

        pos = DT.dataTransfer(locator, 7, dataset, pos, 1);

        pos = DT.dataTransfer(&temp, 2, dataset, pos, 1);

// poll ZDA dataset
        clearInputBuffer();
        gpsSerial.println(F("$EIGPQ,ZDA*39"));
        uint8_t h,m,s,d,mt,y;
        if(receiveDataset() && buffer[findSeparator(1)]!=',') {
// parse time-string and set system clock
          base = findSeparator(1);
          h = decStringToByte(base, 2);
          base += 2;
          m = decStringToByte(base, 2);
          base += 2;
          s = decStringToByte(base, 2);
          base = findSeparator(2);
          d = decStringToByte(base, 2);
          base = findSeparator(3);
          mt = decStringToByte(base, 2);
          base = findSeparator(4)+2;
          y = decStringToByte(base, 2);

          setTime(h,m,s,d,mt,y);
          t = now();

// transfer data into array (part 2)
          pos = DT.dataTransfer(&t, 4, dataset, pos, 1);
          
          DT.transmitData(dataset); // latency to actual system time is about 9ms, mainly caused by serial communication with GPS
          flashLED(1, 250); // flashing 1x indicates transmission of dataset 0

          current_dataset = 0; // send all other data only if GPS position and time fix are positive
        }
      }
      // no position fix -> flash LED 10x at higher rate
      else {
        flashLED(10, 62);
      }
    }
    else {

// poll VTG-data
      sog = getSpeedOverGround();

// Create a dataset (pointer to data, id, size)
      dset_t dataset = DT.createDataset(data, 1, 35);

// transfer data into array (part 1)
      pos = 0;
      pos = DT.dataTransfer(&sog, 1, dataset, pos, 1);

// Calculate Sun's & Moon's rise/set times based on the current coordinates
      time_t tRS[2]; // rise and set times
      uint8_t is_obsolete;
      time_t cal_time;

      for(uint8_t i=0; i<3; i+=2) { // loop Sun and Moon
        time_t cal_time = t - 90000; // start calculation 25h back to make sure set-time is always in the future
        do {
	      is_obsolete = 0;
          for(uint8_t j=0; j<2; ++j) { // calculate rise and set times
            tRS[j] = MS.RiseSet(j + i, cal_time, 51, 4, 13, 'N', 13, 39, 45, 'E');
            if(!tRS[j]) { // timestamp == 0, hence could not be calculated; skip calculation and return 0 for both
              tRS[0] = 0;
              tRS[1] = 0;
              break;
            }
            else { // set time is in the past; add 13h and redo calculation
              if(j && t>tRS[1]) {
                is_obsolete = 1;
                cal_time += 46800;
              }
            }
          }
        }
        while(is_obsolete==1);

// transfer data into array (part 2)
        pos = DT.dataTransfer(tRS, 8, dataset, pos, 1);
      }

// Calculate Sun's and Moon's current position      
      dset pos_data = MS.Position(t, lat_deg, lat_min, lat_sec, lat_o, long_deg, long_min, long_sec, long_o);
      
// calculate the Sun's azimuth in degree*100 (North = 0, East = 90, South = 180, West = 270)
      int32_t azs  = MS.radToDegree(pos_data.x[0]);
// calculate the Sun's altitude in degree*100
      int16_t alts = MS.radToDegree(pos_data.x[1]);

// calculate the Moon's azimuth in degree*100
      int32_t azm  = MS.radToDegree(pos_data.x[2]);
// calculate the Moon's altitude in degree*100
      int16_t altm_topoc = MS.radToDegree(pos_data.x[3]);

// calculate the Moon's phase in %*100 (positive for increasing phase)
      int16_t phasem = 100.0*pos_data.x[4];
      
// transfer data into array (part 3)
      pos = DT.dataTransfer(&t, 4, dataset, pos, 1);
      pos = DT.dataTransfer(&azs, 4, dataset, pos, 1);
      pos = DT.dataTransfer(&alts, 2, dataset, pos, 1);
      pos = DT.dataTransfer(&azm, 4, dataset, pos, 1);
      pos = DT.dataTransfer(&altm_topoc, 2, dataset, pos, 1);
      DT.dataTransfer(&phasem, 2, dataset, pos, 1);
  
// transmit
      DT.transmitData(dataset);
      flashLED(2, 250); // flashing 2x indicates transmission of dataset 2

      current_dataset = 1;
    }
    
    // task has been executed -> set counter to 0
    transmit_data_loop_counter = 0;
  }

// increment loop counter of all cyclical tasks
  ++transmit_data_loop_counter;

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
