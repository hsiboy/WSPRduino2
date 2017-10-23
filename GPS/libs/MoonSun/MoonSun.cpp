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

// ######################################################################################################################
//includes
// ######################################################################################################################

#include <MoonSun.h>

// ######################################################################################################################
//declarations
// ######################################################################################################################

// ######################################################################################################################
//global functions
// ######################################################################################################################

// ######################################################################################################################
//private functions
// ######################################################################################################################
// corrects a given angle to 0...2Pi by adding/subtracting multiples of 2Pi

double MoonSunClass::corrAngle(double angle) {
  const double two_Pi = M_PI*2;

  int32_t factor = angle/two_Pi;
  angle -= factor*two_Pi;
  if(factor<0) {
    angle += two_Pi;
  }

  return angle;
}

// ######################################################################################################################
// calculate Distance and True Anomaly from Eccentricity, Mean Anomaly and Mean Distance

dset MoonSunClass::calDaTA(double e, double M, double a) {
  dset dummy;

  double E = M+e*sin(M)*(1+e*cos(M));
  double x = a*(cos(E)-e);
  double y = a*sqrt(1-square(e))*sin(E);
  dummy.x[0] = atan2(y, x);
  dummy.x[1] = sqrt(square(x)+square(y));

  return dummy;
}

// ######################################################################################################################
// returns the UTC in seconds for a given timestamp

uint32_t MoonSunClass::getUTC_s(time_t time) {
  return 3600*((uint32_t) hour(time)) + 60*((uint32_t) minute(time)) + second(time);
}

//#######################################################################################################################
//public functions
// ######################################################################################################################

// converts radians to degrees
// returns the result *100 as an int32_t
int32_t MoonSunClass::radToDegree(double angle) {
  return 18000*(angle/M_PI);
}

// ######################################################################################################################

/*
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
*/

dset MoonSunClass::Position(time_t time, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o, uint8_t long_deg,
                            uint8_t long_min, uint8_t long_sec, uint8_t long_o) {

  dset dummy;

// calculate day number
  double ut = hour(time) + minute(time)/60.0 + second(time)/3600.0;
  uint8_t d = day(time);
  uint16_t m = month(time);
  uint32_t yr = year(time);
  uint32_t dn = 367*yr - ((7*(yr+(m+9)/12))>>2) + (275*m)/9 + d - 730530;
  double day_number = dn + ut/24.0;

// ~~~~~~~~~~~~~~~~~ Sun ~~~~~~~~~~~~~~~

// calculate the Sun's orbital elements
  //double Ns = 0.0;// Long asc. node
  //double is = 0.0;// Inclination
  double ws = corrAngle(4.9382415669 + 8.21936631E-7*day_number);// Arg. of perigee
  double as = 1.0;// Mean distance (AU)
  double es = 1.6709E-2 - 1.151E-9*day_number;// Eccentricity
  double Ms = corrAngle(6.214192442 + 1.72019696192896E-2*day_number);// Mean anomaly

  dummy = calDaTA(es, Ms, as);

  double lons = dummy.x[0] + ws;

  double xs = dummy.x[1]*cos(lons);
  double ys = dummy.x[1]*sin(lons);

// calculate obliquity of the ecliptic
  double oblecl = 4.090929594E-1 - 6.218608E-9*day_number;

  double zs = ys*sin(oblecl);
  ys = ys*cos(oblecl);

  double RAs = atan2( ys, xs );
  double Decs = atan2( zs, sqrt(square(xs) + square(ys)) );

// calculate local longitude in h and local latitude in radians
  double llo = long_deg/15.0 + long_min/900.0 + long_sec/54000.0;
  if(long_o=='W' || long_o=='w') { llo *= -1; }

  double lla = (lat_deg + lat_min/60.0 + lat_sec/3600.0)*M_PI_180;
  if(lat_o=='S' || lat_o=='s') { lla *= -1; }

// calculate the Sun's mean longitude
  double Ls = ws + Ms;
// calculate siderial time in h
  double GMST0 = corrAngle(Ls + M_PI)*_12_M_PI;
  double LST = GMST0 + ut + llo;

// calculate the Sun's azimuth and altitude
  double HAs = LST*M_PI_12 - RAs;

  dummy.x[0] = sin(lla);
  dummy.x[1] = cos(lla);
  dummy.x[2] = cos(Decs);
  dummy.x[3] = cos(HAs)*dummy.x[2];
  ys = sin(HAs)*dummy.x[2];
  dummy.x[4] = sin(Decs);

  xs = dummy.x[3]*dummy.x[0] - dummy.x[4]*dummy.x[1];
  zs = dummy.x[3]*dummy.x[1] + dummy.x[4]*dummy.x[0];

// calculate the Sun's azimuth in radians (North = 0, East = PI/2, South = PI, West = 3PI/2)
  double azs  = atan2( ys, xs ) + M_PI;
// calculate the Sun's altitude in radians
  double alts = atan2( zs, sqrt(square(xs)+square(ys)));

// ~~~~~~~~~~~~~~~~~ Moon ~~~~~~~~~~~~~~~

// calculate the Moon's orbital elements incl. perturbations
  double Nm = corrAngle(2.1838048293 - 9.24218306302609E-4*day_number);// Long asc. node
  double im = 8.98041713E-2;// Inclination
  double wm = corrAngle(5.5512535601 + 2.86857642388938E-3*day_number);// Arg. of perigee
  double am = 60.2666;// Mean distance (Earth radii)
  double em = 5.49E-2;// Eccentricity
  double Mm = corrAngle(2.0135060729 + 2.280271437424892E-1*day_number);// Mean anomaly
// Perturbations
  double Lm = Nm + wm + Mm;
  double Dm = Lm - Ls;
  double Fm = Lm - Nm;

// calculate the Moon's geocentric position
  dummy = calDaTA(em, Mm, am);
  double vm = dummy.x[0];
  double rm = dummy.x[1];

  vm += wm;
  dummy.x[0] = sin(Nm);
  dummy.x[1] = cos(Nm);
  dummy.x[2] = cos(vm);
  dummy.x[3] = sin(vm);
  dummy.x[4] = cos(im)*dummy.x[3];

  double xm = rm*(dummy.x[1]*dummy.x[2] - dummy.x[0]*dummy.x[4]);
  double ym = rm*(dummy.x[0]*dummy.x[2] + dummy.x[1]*dummy.x[4]);
  double zm = rm*dummy.x[3]*sin(im);

  double lom  = atan2( ym, xm ); // Ecliptical longitude
  double lam = atan2( zm, sqrt(square(xm)+square(ym)) ); // Ecliptical latitude

  double a = 2*Dm;
  double b = Mm - a;
// calculate Perturbations in longitude and correct ecliptic position
  dummy.x[0] = -2.2235495E-2 * sin(b);// Evection
  dummy.x[1] = 1.1484266E-2 * sin(a);// Variation)
  dummy.x[2] = -3.2463124E-3 * sin(Ms);// Yearly equation
  dummy.x[3] = -1.0297443E-3 * sin(2*Mm - a);
  dummy.x[4] = -9.9483767E-4 * sin(b + Ms);
  dummy.x[5] = 9.250245E-4 * sin(Mm + a);
  dummy.x[6] = 8.0285146E-4 * sin(a - Ms);
  dummy.x[7] = 7.1558499E-4 * sin(Mm - Ms);
  dummy.x[8] = -6.1086524E-4 * sin(Dm);// Parallactic equation
  dummy.x[9] = -5.4105207E-4 * sin(Mm + Ms);
  dummy.x[10] = -2.6179939E-4 * sin(2*Fm - a);
  dummy.x[11] = 1.9198622E-4 * sin(Mm - 4*Dm);

  uint8_t i;
  for(i=0; i<12; ++i) { lom += dummy.x[i]; }

// calculate Perturbations in latitude
  dummy.x[0] = -3.0194196E-3 * sin(Fm - a);
  dummy.x[1] = -9.5993109E-4 * sin(b - Fm);
  dummy.x[2] = -8.0285146E-4 * sin(b + Fm);
  dummy.x[3] = 5.7595865E-4 * sin(Fm + a);
  dummy.x[4] = 2.9670597E-4 * sin(2*Mm + Fm);

  for(i=0; i<5; ++i) { lam += dummy.x[i]; }

// calculate Perturbations in lunar distance (Earth radii)
  double pld = -0.58*cos(b) - 0.46*cos(a);
  rm += pld;

// calculate geocentric coordinates
  dummy.x[0] = cos(lam);
  xm = rm * cos(lom)*dummy.x[0];
  ym = rm * sin(lom)*dummy.x[0];
  zm = rm * sin(lam);

// convert to equatorial coordinates and calculate RA & Declination
  dummy.x[0] = ym;
  dummy.x[1] = zm;
  dummy.x[2] = sin(oblecl);
  dummy.x[3] = cos(oblecl);
  ym = dummy.x[0]*dummy.x[3] - dummy.x[1]*dummy.x[2];
  zm = dummy.x[0]*dummy.x[2] + dummy.x[1]*dummy.x[3];

  double RAm  = atan2( ym, xm );
  double Decm = atan2( zm, sqrt(square(xm)+square(ym)) );

  // calculate the Moon's azimuth and altitude
  double HAm = LST*M_PI_12 - RAm;

  dummy.x[0] = cos(Decm);
  xm = cos(HAm)*dummy.x[0];
  ym = sin(HAm)*dummy.x[0];
  zm = sin(Decm);

  dummy.x[0] = sin(lla);
  dummy.x[1] = cos(lla);
  dummy.x[2] = xm;
  dummy.x[3] = zm;
  xm = dummy.x[2]*dummy.x[0] - dummy.x[3]*dummy.x[1];
  zm = dummy.x[2]*dummy.x[1] + dummy.x[3]*dummy.x[0];

// calculate the Moon's azimuth and altitude in radians
  double azm  = atan2( ym, xm ) + M_PI;
  double altm_geoc = atan2( zm, sqrt(square(xm)+square(ym)) );

// calculate the Moon's topocentric position (altitude correction only)
  double mpar = asin( 1/rm );
// calculate the Moon's altitude in radians
  double altm_topoc = altm_geoc - mpar*cos(altm_geoc);

// calculate the Moon's phase in %
  dummy.x[0] = lons - lom;
  dummy.x[1] = cos(lam);
  double elongm = acos( cos(dummy.x[0])*cos(lam) );
  double FVm = M_PI - elongm;
  double phasem = copysign(50.0*(1+cos(FVm)), -sin(dummy.x[0])*dummy.x[1]);

// output
  dummy.x[0] = azs;
  dummy.x[1] = alts;
  dummy.x[2] = azm;
  dummy.x[3] = altm_topoc;
  dummy.x[4] = phasem; // is positive for increasing phase
  // some support-parameter required for rise/set calculations
  dummy.x[5] = RAs;
  dummy.x[6] = Decs;
  dummy.x[7] = RAm;
  dummy.x[8] = Decm;
  dummy.x[9] = GMST0;
  dummy.x[10] = llo;
  dummy.x[11] = lla;
  dummy.x[12] = altm_geoc;

  return dummy;
}

//################################################################################################################

/*
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
*/

time_t MoonSunClass::RiseSet(uint8_t selection, time_t time, uint8_t lat_deg, uint8_t lat_min, uint8_t lat_sec, uint8_t lat_o,
		             uint8_t long_deg, uint8_t long_min, uint8_t long_sec, uint8_t long_o) {

  time_t time_cal;
  int32_t ut_South, ut_RS, ut_RS_last;
  int8_t direction = -1;
  uint8_t loop_limit = 10;
  double RA;
  double Dec;
  double GMST0;
  double llo, lla;
  dset pos_data;
  double LHA;
  double h; // the reference altitude

// calculate rise/ set times (iteration)
  ut_RS = getUTC_s(time);
  time -= ut_RS;

// begin iteration
  do {
	--loop_limit;
	ut_RS_last = ut_RS;
	time_cal = time + ut_RS;
	
	pos_data = Position(time_cal, lat_deg, lat_min, lat_sec, lat_o, long_deg, long_min, long_sec, long_o);

	if(selection & 2) {
	  h = -0.014828317 + pos_data.x[12] - pos_data.x[3]; // Moon's upper limb touches the horizon
		                                          // atmospheric refraction and parallax accounted for
	  RA = pos_data.x[7]; // RAm
	  Dec = pos_data.x[8]; // Decm
	}
	else {
	  h = -0.014538593; //Sun's upper limb touches the horizon; atmospheric refraction accounted for
	  RA = pos_data.x[5]; // RAs
	  Dec = pos_data.x[6]; // Decs
	}
	GMST0 = pos_data.x[9];
	llo = pos_data.x[10];
	lla = pos_data.x[11];

	ut_South = 3600*(RA*_12_M_PI - GMST0 - llo + 24);  // calculate objects time of transit

	LHA = (sin(h) - sin(lla)*sin(Dec))/(cos(lla)*cos(Dec));  // calculate local hour angle
	if(LHA < 1.0 && LHA > -1.0) { // >1 == never rises / <-1 == never sets
	  LHA = acos(LHA)*_12_M_PI;
	  if(selection & 1) { direction = 1; } // rise or set
	  ut_RS = ut_South + LHA*(3600*direction);
	}
	else { // no event found
	  time_cal = 0;
	  break;
	}

	if(!loop_limit) { // just in case the iteration gets stuck under some unusual circumstances (i.x[4]. beginning or end
		              // of Midnight-Sun
	  time_cal = 0;
	  break;
	}
  }
  while(abs(ut_RS - ut_RS_last) > 9); // iterate to +/-10 seconds accuracy

  return time_cal;
}

MoonSunClass MS;
