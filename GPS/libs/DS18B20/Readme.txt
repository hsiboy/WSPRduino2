"DS18B20"
Arduino-library supporting communication with a single Maxim DS18B20 temperature sensor in 3-wire setup
(separate power supply and 4.7k pullup resistor)

V1.1 / Copyright (C) 2017, T.Rode (DL1DUZ)

Permission is granted to use, copy, modify, and distribute this software
and documentation for non-commercial purposes.

Required ressources:
 - 1 digital input pin

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Methods:

uint8_t init(const uint8_t PIN, const uint8_t RESOLUTION)

initializes the library; resolution can be 9...12 bit corresponding to 0.5°C, 0.25°C, 0.125°C, 0.0625°C
returns 1 if successful, otherwise returns 0

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t convertTemp(const uint8_t IS_STALLING)

initiates temperature conversion; wait > 93.75*2^(RESOLUTION - 9) ms before attempt reading T
if "IS_STALLING" is True, the function will stall until conversion is done (timeout = 800ms)
returns 1 if successful, otherwise returns 0

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int16_t getTemp()

reads the temperature;
returns actual temperature*16 as an int16_t or 0x8000 == -32768 in case of error

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Simple Arduino example code: Cyclical output of data from DS18B20 connected to pin12.
*/

#include <DS18B20.h>

//######################################################################################################

// variables and constants
const uint8_t TEMP_SENSOR_PIN = 12;
const uint8_t RESOLUTION = 12; // 12 bit resolution == 0.0625°C

//######################################################################################################

void setup() {

// init serial communication to PC via USB/UART
  Serial.begin(9600);
  
// init temperature sensor
  if(!TS.init(TEMP_SENSOR_PIN, RESOLUTION)) {
    Serial.println(F("Initializing temperature sensor failed."));
    while(true) { delay(1); } // eternal loop
  }
  
}

//######################################################################################################

void loop() {
  
// convert temperature
  if(TS.convertTemp(1)) {
// read temperature
    int16_t temp = TS.getTemp();
    if(temp == 0x8000) {
      Serial.println(F("Temperature readout failed."));
    }
    else {
// output temperature to PC with 0.1°C accuracy
      Serial.print(F("Current temperature is: "));
      Serial.print(((float) temp)/16, 1);
      Serial.println(F("°C"));
    }
  }
  else {
    Serial.println(F("Temperature conversion failed."));
  }

}