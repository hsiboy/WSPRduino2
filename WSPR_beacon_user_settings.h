// Beginning user settings

// call and power[mW]
const char CALL[7] = "DL1DUZ";  // 6 character callsign; 3rd character is forced to be a number; fill all blanks with " "
const uint16_t POWER = 100;     // Power[mW] from 1 to 9999

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// the number of available bands (currently 10; 160-10m)
const uint8_t BAND_COUNT = 10;

// base-frequencies for all bands (some may depend on your national frequqncy allocations)
const uint32_t BASE_FREQUENCY[BAND_COUNT] PROGMEM = {1838000,3594000,5366100,7040000,10140100,14097000,18106000,
                                                     21096000,24926000,28126000};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// The status of the bands from 160m-10m (enabled/disabled) is no longer defined at this point.

// This parameter can now be interactively modified during startup of the beacon by connecting the Arduino to a PC running
// a terminal program at 9600 baud. For the beacon to enter the setup dialog the "transmitter disabled" switch has to be
// set to "TX off" prior to power-up. The terminal software implemented at the Arduino programming environment
// (Tools -> Serial Monitor) will do the job. I also tested kitty_portable from "9bis software". In order to save you the
// setup I added a copy of that program including the ini-file to this package. Simply start the "exe", load the WSPR-beacon
// setup, set the right com-port (e.g. COM 1 2 3 ...) and open the connection. At the end of this file you'll find a
// screenshot showing how things should look like if all settings are ok.

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
// beacon operating mode /// 0 = TX frequency will be randomly chosen within the band limits at each new
// transmission cycle; 1...194 = TX frequency will be fixed at "lower band limit" + 1...194 Hz
uint8_t beacon_mode = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// The beacon's duty cycle is no longer defined at this point but interactively modified during startup (see above).

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// the (actual) DDS clock frequency
const uint32_t DDS_CLK = 124999170;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// "SWR meter installed?" (0 = no, 1 = yes)
const uint8_t SWR_METER_INSTALLED = 1;

// sets the reference for analog inputs (INTERNAL = 1.1V, DEFAULT = 5V)
const uint8_t ref = INTERNAL;

// SWR-meter polynomial (correcting nonlinearities of the SWR-gauge / must be adjusted depending on the
// diodes used); SWR_x represents the output of the ADC and SWR_y the (corrected) value times 32
const uint16_t SWR_X[18] PROGMEM = {0,2,4,18,37,65,93,140,186,279,372,465,558,651,744,837,930,1023};
const uint16_t SWR_Y[18] PROGMEM = {0,261,442,1404,2441,3763,4954,6783,8436,11518,14365,17050,19612,22077,24460,26784,
                                    29760,32736};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// sets the timezone (offset to UTC) /// (0 = GMT, 1 = CET, 2 = EET)
const int8_t TIMEZONE = 2;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// sets the temperature scale /// (0 = Celsius, 1 = Fahrenheit)
const uint8_t TEMP_SCALE = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// sets the speed unit /// (0 = km/h, 1 = mph, 2 = kn)
const uint8_t SPEED_UNIT = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// sets the distance unit /// (0 = m, 1 = ft)
const uint8_t DIST_UNIT = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// enable/disable some extra screens (0 = off, 1 = on)
const uint8_t COORDINATES = 1; // shows a screen displaying the current longitude and latitude
const uint8_t ALT_SPEED = 1; // shows a screen displaying the current altitude and speed over ground

// Sorry, but having access to exact time and coordinates I couldn't resist giving the beacon some basic skills in
// astromechanics. So if you're interested in the heavens above, you may want to activate those features below.
const uint8_t SUN = 1; // shows a screen displaying time of sunrise & sunset
const uint8_t MOON = 1; // shows a screen displaying time of moonrise & moonset plus the Moon's phase (e.g. 53% decreasing)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// port assignment

// SWR-meter (analog pins)
const uint8_t SWR_REF_PIN = A4;
const uint8_t SWR_FWD_PIN = A5;

// Transmitter disabled (digital pin)
const uint8_t TRANSMITTER_DISABLED_PIN = 3;

// HD44780 (digital pins)
const uint8_t RS_PIN = 13;
const uint8_t ENABLE_PIN = 12;
const uint8_t D4_PIN = 11;
const uint8_t D5_PIN = 10;
const uint8_t D6_PIN = 9;
const uint8_t D7_PIN = 8;
const uint8_t LED = 1;

// GPS-module receiver (digital pin capable of triggering interrupts)
const uint8_t GPS_INPUT_PIN = 2;

// End user settings

/*
WSPR beacon setup dialog. (Demo)

Current band settings:

160m is on
80m is off
60m is on
40m is off
30m is on
20m is off
17m is on
15m is off
12m is on
10m is off

Would you like to change these settings (y/n)? YES

Activate 160m (y/n)? NO
Activate 80m (y/n)? NO
Activate 60m (y/n)? NO
Activate 40m (y/n)? NO
Activate 30m (y/n)? NO
Activate 20m (y/n)? YES
Activate 17m (y/n)? NO
Activate 15m (y/n)? NO
Activate 12m (y/n)? NO
Activate 10m (y/n)? NO

The beacon's duty cycle is 1/4 = 25.0%.
To increase/decrease (repeatedly) press 'i' or 'd', to quit press 'q'.
The beacon's duty cycle is 1/3 = 33.3%.
The beacon's duty cycle is 1/2 = 50.0%.
The beacon's duty cycle is 1/1 = 100.0%.

Would you like to discard changes and repeat setup? (y/n)? NO

Done! Settings have been saved. You may now disconnect and restart the beacon.
*/
