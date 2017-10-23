/*
  "MoonSun"
  calculates Moon's and Sun's position, time of rise and set + Moon's phase for a given UTC (time_t) and coordinate
  with "double"-precision, if available

  derived from Paul Schlyter's http://www.stjarnhimlen.se

  Requires Michael Margolis library "Time"
  
  V1.1 / Copyright (C) 2017, T.Rode (DL1DUZ)

  Permission is granted to use, copy, modify, and distribute this software
  and documentation for non-commercial purposes.
*/

#ifndef MoonSun_h_
#define MoonSun_h_

#if (ARDUINO >= 100)
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

#include <stdint.h>
#include <math.h>
#include <TimeLib.h>

//################################################################################################################
//definitions
//################################################################################################################

// structure containing an array of 13 "double"-variables
struct dset {
  double x[13];
};

//################################################################################################################

class MoonSunClass {

public:

  dset Position(time_t time, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o, uint8_t long_deg,
                uint8_t long_min, uint8_t long_sec, uint8_t long_o);
  time_t RiseSet(uint8_t selection, time_t time, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o,
		 uint8_t long_deg, uint8_t long_min, uint8_t long_sec, uint8_t long_o);
  int32_t radToDegree(double angle);

private:

  // some constants
  const double M_PI_12 = M_PI/12;
  const double M_PI_180 = M_PI/180;
  const double _12_M_PI = 12/M_PI;

  double corrAngle(double angle);
  dset calDaTA(double e, double M, double a);
  uint32_t getUTC_s(time_t time);

};

extern MoonSunClass MS;

#endif // MoonSun_h_
