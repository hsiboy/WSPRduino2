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

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Datatypes:

// structure containing an array of 13 "double"-variables
struct dset {
  double x[13];
};

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Methods:

int32_t radToDegree(double angle)

converts radians to degrees
returns the result *100 as an int32_t

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

dset Position(time_t time, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o, uint8_t long_deg,
              uint8_t long_min, uint8_t long_sec, uint8_t long_o)

  - calculates Moon's and Sun's position + Moon's phase for a given UTC (time_t) and coordinate with "double"-precision,
    if available
  - derived from Paul Schlyter's http://www.stjarnhimlen.se

  - input parameter:

    time -> the timestamp (UTC)
    lat_deg -> the observers latitude in degree
    lat_min -> the observers latitude in minutes
    lat_sec -> the observers latitude in seconds
    lat_o -> the observers latitude orientation or direction ('N', 'n', 'S', 's')
    long_deg -> the observers longitude in degree
    long_min -> the observers longitude in minutes
    long_sec -> the observers longitude in seconds
    long_o -> the observers longitude orientation or direction ('E', 'e', 'W', 'w')

  - returns an element of the type "dset" containing:

    dset.x[0] = azimuth Sun [radians] (North = 0, East = PI/2, South = PI, West = 3PI/2)
    dset.x[1] = altitude Sun [radians] (negative if set)
    dset.x[2] = azimuth Moon [radians] (North = 0, East = PI/2, South = PI, West = 3PI/2)
    dset.x[3] = altitude Moon [radians] (negative if set)
    dset.x[4] = the Moon's phase [%] (positive for increasing phase)
    - some support-parameter required for rise/set calculations
    dset.x[5] = RAs
    dset.x[6] = Decs
    dset.x[7] = RAm
    dset.x[8] = Decm
    dset.x[9] = GMST0
    dset.x[10] = llo
    dset.x[11] = lla
    dset.x[12] = altm_geoc [radians]

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

time_t RiseSet(uint8_t selection, time_t time, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o,
	       uint8_t long_deg, uint8_t long_min, uint8_t long_sec, uint8_t long_o)

  - calculates rise/set-times for a given object, day (time_t) and coordinate
  - derived from Paul Schlyter's http://www.stjarnhimlen.se

  - input parameter:
    selection -> 0 == Sunrise, 1 == Sunset, 2 == Moonrise, 3 == Moonset
    time -> the timestamp to start calculation at (typically now())
    lat_deg -> the observers latitude in degree
    lat_min -> the observers latitude in minutes
    lat_sec -> the observers latitude in seconds
    lat_o -> the observers latitude orientation or direction ('N', 'n', 'S', 's')
    long_deg -> the observers longitude in degree
    long_min -> the observers longitude in minutes
    long_sec -> the observers longitude in seconds
    long_o -> the observers longitude orientation or direction ('E', 'e', 'W', 'w')

  - returns the timestamp (UTC) of the event requested [time_t] or 0, if event could not be resolved

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Simple Arduino demo-code: Cyclically calculate and print current positions as well as rise- and
set-times for the author's home-coordinates / the system clock should be synchronized to UTC
*/

#include <MoonSun.h>

//###################################################################################################

void setup() {
  Serial.begin(9600);
}

//###################################################################################################

void loop() {
  time_t t = now() + 1505315746;

  dset pos_data = MS.Position(t, 51, 4, 13, 'N', 13, 39, 45, 'E');

// calculate the Sun's azimuth in degree*100 (North = 0, East = 90, South = 180, West = 270)
  int32_t azs  = MS.radToDegree(pos_data.x[0]);
// calculate the Sun's altitude in degree*100
  int16_t alts = MS.radToDegree(pos_data.x[1]);

// calculate the Moon's azimuth in degree*100
  int32_t azm  = MS.radToDegree(pos_data.x[2]);
// calculate the Moon's altitude in degree*100
  int16_t altm_topoc = MS.radToDegree(pos_data.x[3]);

// calculate the Moon's phase in % (positive for increasing phase)
  int8_t phasem = pos_data.x[4];

  Serial.print(F("Timestamp: "));Serial.print(hour(t));Serial.print(F(":"));Serial.print(minute(t));Serial.print(F(":"));
  Serial.print(second(t));Serial.print(F("   "));Serial.print(month(t));Serial.print(F("/"));Serial.print(day(t));
  Serial.print(F("/"));Serial.println(year(t));
  Serial.print(F("Sun's azimuth [degree]: "));Serial.print(((float) azs)/100.0, 1);Serial.print(F("   "));
  Serial.print(F("Sun's altitude [degree]: "));Serial.print(((float) alts)/100.0, 1);Serial.println();
  Serial.print(F("Moon's azimuth [degree]: "));Serial.print(((float) azm)/100.0, 1);Serial.print(F("   "));
  Serial.print(F("Moon's altitude [degree]: "));Serial.print(((float) altm_topoc)/100.0, 1);Serial.print(F("   "));
  Serial.print(F("Moon's phase: "));Serial.print(abs(phasem), 1);Serial.print(F("% "));
  if(pos_data.x[4] > 0.0) {Serial.print(F("in"));} else {Serial.print(F("de"));};Serial.println(F("creasing"));
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  
// calculate Sun's and Moon's rise/set times
// set-time will always be in the future

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

    // output results
    if(i&2) {Serial.print(F("Moon"));} else {Serial.print(F("Sun"));};
    Serial.print(F("rise: "));Serial.print(hour(tRS[0]));Serial.print(F(":"));Serial.print(minute(tRS[0]));
    Serial.print(F(":"));Serial.print(second(tRS[0]));Serial.print(F("   "));Serial.print(month(tRS[0]));
    Serial.print(F("/"));Serial.print(day(tRS[0]));Serial.print(F("/"));Serial.println(year(tRS[0]));
    if(i&2) {Serial.print(F("Moon"));} else {Serial.print(F("Sun"));};
    Serial.print(F("set: "));Serial.print(hour(tRS[1]));Serial.print(F(":"));Serial.print(minute(tRS[0]));
    Serial.print(F(":"));Serial.print(second(tRS[1]));Serial.print(F("   "));Serial.print(month(tRS[1]));
    Serial.print(F("/"));Serial.print(day(tRS[1]));Serial.print(F("/"));Serial.println(year(tRS[1]));
  }

  Serial.println(F("#######################################################################################################"));

  delay(10000); // next loop in 10s
}
