/*
 WSPR_beacon
 V2.2 (GPS-based time and position aquisition)
 Copyright (C) 2017
 Thomas Rode / DL1DUZ
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 Arduino-based WSPR beacon
 
 Generates WSPR coordinated frequency hopping transmissions 
 on 10 thru 160 meters using an AD9850 DDS.

 Time synchronisation and calculation of QTH-locator are based on GPS.
*/

#include<limits.h>

// Library providing basic WSPR functionality
#include <WSPR.h>
#include <AD9850.h>
// Receiver-library to listen to transmissions from GPS-module
#include <DataReceiver.h>
#include <util/atomic.h>

#include <LiquidCrystal.h>
#include <TimeLib.h>

#include "WSPR_beacon_user_settings.h"

//##########################################################################################################

// build AD9850-instance dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN, CLOCK_FREQUENCY[Hz])
AD9850 dds(7, 6, 5, 4, DDS_CLK);

/* Variables/constants available at the LCD module
32  SP  64  @ 96  `
33  !  65 A 97  a
34  "  66 B 98  b
35  #  67 C 99  c
36  $  68 D 100 d
37  %  69 E 101 e
38  &  70 F 102 f
39  '  71 G 103 g
40  (  72 H 104 h
41  )  73 I 105 i
42  *  74 J 106 j
43  +  75 K 107 k
44  ,  76 L 108 l
45  -  77 M 109 m
46  .  78 N 110 n
47  /  79 O 111 o
48  0  80 P 112 p
49  1  81 Q 113 q
50  2  82 R 114 r
51  3  83 S 115 s
52  4  84 T 116 t
53  5  85 U 117 u
54  6  86 V 118 v
55  7  87 W 119 w
56  8  88 X 120 x
57  9  89 Y 121 y
58  :  90 Z 122 z
59  ;  91 [ 123 {
60  <  92 ¥ 124 |
61  =  93 ] 125 }
62  >  94 ^ 126 →
63  ?  95 _ 127 ←

253 ÷ 244 Ω 223 °
*/

// some LCD display messages

const uint8_t SERIAL_DISP[33] PROGMEM = "USB @ 9600 Baud ready for setup ";
const uint8_t BAND_INFO[33] PROGMEM = "160 80 60 40 30  20 17 15 12 10 ";
const uint8_t SETTINGS_NOT_VALID[33] PROGMEM = "invalid settings / system halted";
const uint8_t INTRO[33] PROGMEM = " WSPR-beacon by DL1DUZ  160m-10m";
const uint8_t USER_DATA[33] PROGMEM = "------ ----[--] Power:   -mW    ";
const uint8_t TX_STANDBY[33] PROGMEM = " WSPR-beacon is  on standby     ";
const uint8_t TX_ON_AIR[33] PROGMEM = "TX is on air at  --m / SWR:-.-  ";
const uint8_t TX_SWR_DISABLED[33] PROGMEM = "TX disabled due to SWR>3 at  --m";
const uint8_t TX_GPS_DISABLED[33] PROGMEM = "no GSP-data ->  beacon disabled ";
const uint8_t TX_HARDWARE_DISABLED[33] PROGMEM = "TX switched off by user         ";
const uint8_t WAITING_FOR_GPS[33] PROGMEM = "waiting for datafrom GPS-module ";
const uint8_t TIME_DATE[33] PROGMEM = "--:--:--      C ---. --.--.---- ";
const uint8_t WEEKDAY_CODING[22] PROGMEM = "SunMonTueWedThuFriSat";
const uint8_t ALT_SPEED_DISP[33] PROGMEM = "Altitude:    -m Speed:   -km/h  ";
const uint8_t SUN_DISP[33] PROGMEM = "Sun: today --:--     today --:--";
const uint8_t MOON_DISP[33] PROGMEM = "Moon:today --:--     today --:--";
const uint8_t MOON_PHASE_DISP[33] PROGMEM = "  Moon's phase:    -% decreasing";

// build LiquidCrystal-instance
LiquidCrystal lcd(RS_PIN, ENABLE_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);

// the number of characters at the LCD-display
const uint8_t DISPLAY_SIZE = 32;

// the content matrix lcd_content[position] and its last image
// character 0...15 for line 1 and 16...31 for line 2
uint8_t lcd_content[DISPLAY_SIZE];
uint8_t lcd_content_last[DISPLAY_SIZE] = {0};

// the current cursor position
uint8_t cursor_position = 0;
// current cursor status (0==nothing; 1==only corsor; 2==only blinking; 3==blinking cursor)
uint8_t cursor_status = 0;

// a pointer determining the active display content and its max. value
const uint8_t MAX_DISP_CONTENT = 7;
uint8_t disp_content_pointer = 0;
// a pointer determining the content to be on fast refresh
uint8_t fast_refresh_pointer = 0;

// the current LED-backlight brightness, the min value and change per step
const uint8_t MIN_BRIGHTNESS = 30;
uint8_t brightness;
int8_t brightness_change;
uint8_t pulsing_on;
uint8_t backlight_on_time_counter;

// Variables/constants used by GPS
uint8_t lat_long[4][2]; // longitude and latitude (lat_d, lat_m, lat_s, lat_o, long_d, long_m, long_s, long_o)
uint16_t alt; // altitude
uint8_t sog; // speed over ground
time_t rs[4] = { 0 }; // Sun & Moon rise- and set-times
int16_t phase; // the Moon's phase

// Variables/constants used by the thermometer

int16_t temp = 0;

// Variables/constants used by WSPR transmitter

char locator[5] = "JO61"; // Use 4 character locator e.g. "EM64"
char loc[3] = "TB"; // remaining 2 digits of the locator to get 6 digit resolution
// status of the bands (1 = enabled / 0 = disabled)
uint8_t band_status[BAND_COUNT] = { 0 };
// to control the beacon's duty cycle
uint8_t beacon_idle_level;
uint8_t beacon_idle_counter = 0;

time_t gps_time;
time_t last_sync = 0;

// Flags indicating the status of the transmitter (active and hardware-disabled).
uint8_t on_air;
uint8_t td;
uint8_t td_acknowledged = 0;
// Flag indicating that GPS-based data is valid
uint8_t gps_valid = 0;
// Array holding the coded band-information to be displayed on LCD
char band[4];
// array holding the delta phase values for WSPR lower band limit @ 160m-10m
// can be adjusted depending on the actual DDS clock frequency by calling setDeltaPhaseBase()
uint32_t deltaphase_base[BAND_COUNT];
// variables holding the reflected SWR-readings
uint16_t fwd_value;
uint32_t ref_value;
// array holding the 10x the SWR measured at the various bands (0 == SWR was >3 at more than 1 run)
uint8_t swr[BAND_COUNT];
// sum of all SWR-values
uint16_t swr_sum = BAND_COUNT;
// swr sliding average over 10 measurements and a pointer to the current value
const uint8_t SWR_AVG_LENGTH = 10;
uint8_t swr_avg[SWR_AVG_LENGTH];
uint8_t swr_avg_pointer;
// Flag indicating that the SWR-meater is active
uint8_t SWR_check_active = 0;

// the "band switch"
uint8_t band_pointer = BAND_COUNT - 1;

// a pointer to the symbol table
uint8_t symbol_counter;

// Delta phase value representing 194Hz (200Hz - 6Hz signal bandwidth)
const uint16_t DELTAPHASE_BANDWITH = dds.calculatePhaseValue(194);

// Delta phase offset values (to represent 4bit PSK with 1.4648Hz spacing)
const uint8_t DELTAPHASE_PSK[4] = {0, uint8_t(dds.calculatePhaseValue(14648)/10000),
                                   uint8_t(dds.calculatePhaseValue(29296)/10000),
                                   uint8_t(dds.calculatePhaseValue(43944)/10000)};

// the actual phaseword to set the DDS
uint32_t deltaphase;

// variables and constants used by the receiver

const uint8_t BYTE_RATE = 30;

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

// Variables/constants used by the repetitive task scheduler

// a constant specifying the desired loop runtime in µs
// must not be larger then 16383 due to software restrictions
// a proven value is about 100x the shortest task interval
// set to 100µs
const uint8_t TLR = 100;

// a constant specifying the loop cycles between 2 executions of the "process_datasets"-routine
// cycles = interval-time[µs]/TLR
// set to 4s
const uint16_t PROCESS_DATASETS_LOOPS = 4000000/TLR;
// the "process datasets" loop cycle counter
uint16_t process_datasets_loop_counter = 0;

// a constant specifying the loop cycles between 2 executions of the "system timer"-routine
// cycles = interval-time[µs]/TLR
// set to 100ms
const uint16_t SYSTEM_TIMER_LOOPS = 100000/TLR;
// the "system timer" loop cycle counter
uint16_t system_timer_loop_counter = 0;

// the constant specifying the loop cycles between 2 executions of the "single task scheduler"-routine
// cycles = interval-time[µs]/TLR
// set to 11ms
const uint8_t SINGLE_TASK_SCHEDULER_LOOPS = 11000/TLR;
// "single task scheduler" loop cycle counter
uint8_t single_task_scheduler_loop_counter = SINGLE_TASK_SCHEDULER_LOOPS;

// the constant specifying the loop cycles between 2 executions of the "check SWR"-routine
// cycles = interval-time[µs]/TLR
// set to 2.3ms
const uint8_t CHECK_SWR_LOOPS = 2300/TLR;
// "check SWR" loop cycle counter
int16_t check_SWR_loop_counter = 0;

// a constant specifying the loop cycles between 2 executions of the "transmit symbol"-routine
// cycles = interval-time[µs]/TLR
// set to 683ms
const uint16_t TRANSMIT_SYMBOL_LOOPS = 682667/TLR;
// the "transmit symbol" loop cycle counter
int16_t transmit_symbol_loop_counter = TRANSMIT_SYMBOL_LOOPS;

// variables used by the single task scheduling routine

// a timer counting the seconds after system-startup; resolution is 0.1s (10 == 1s)
volatile uint32_t timer = 0;
// a constant holding the max. task-id (in this example 6 tasks from 0...5 are available)
const uint8_t MID = 5;
// an array holding the due-time for all scheduled tasks (<0 == off/no execution)
volatile int32_t tl[MID + 1];

//##########################################################################################################
//##########################################################################################################

// dataset 0 is available -> process data
// keep processing short; no new data will be received while this one is busy!!!
void processDataset_0() {

  uint8_t *data = DR.getDataArray();

  uint8_t shift;
  for(uint8_t i=0; i<2; ++i) {
    shift = i<<2;
    for(uint8_t j=0; j<3; ++j) {
      lat_long[j][i] = *(data + j + shift);
    }
    lat_long[3][i] = char(*(data + 3 + shift));
  }

  sog = *(data+10);
  if(SPEED_UNIT) {
    uint32_t s = sog;
    if(SPEED_UNIT & 1) { // mph instead of km/h
      s *= 621371;
    }
    else { // knots instead of km/h
      s *= 539957;
    }
    sog = s/1000000;
  }

  DR.dataTransfer(&alt, 2, 8, 0);
  if(DIST_UNIT) { // feet instead of meter
    uint32_t a = alt;
    a *= 328084;
    alt = a/100000;
  }

  // read outside temperature from received datastream
  DR.dataTransfer(&temp, 2, 19, 0);
  temp = (temp+8)/16;
  if(TEMP_SCALE) { temp = (9*temp + 160)/5; }

  // read QTH-locator from received datastream
  DR.dataTransfer(locator, 4, 12, 0);
  DR.dataTransfer(loc, 2, 16, 0);
  
  // read GPS-time from received datastream
  DR.dataTransfer(&gps_time, 4, 21, 0);
  uint8_t d_s;
  // calculate time [ms] since dataset was received
  int32_t d_ms = millis() - DR.getTimestamp();
  if(d_ms > 0) { // if delta-t is positive (no counter overflow) ...
    // schedule updating of the "last time synchronisation" timestamp and clock adjustment
    d_s = d_ms/1000 + 1;
    gps_time = gps_time + d_s + TIMEZONE*3600;
    scheduleTask(3, d_s*10 - (d_ms+50)/100);
  }
}

//###########################################################################################################

// dataset 1 is available -> process data
// keep processing short; no new data will be received while this one is busy!!!
void processDataset_1() {
  // read speed over ground from received datastream
  DR.dataTransfer(&sog, 1, 0, 0);

  // read Sun's and Moon's rise/set-times from received datastream
  DR.dataTransfer(rs, 16, 1, 0);

  // read Moon's phase from received datastream
  DR.dataTransfer(&phase, 2, 33, 0);
}

//##########################################################################################################
//##########################################################################################################

void setup() {

  analogReference(ref);
  pinMode(SWR_FWD_PIN, INPUT);
  pinMode(SWR_REF_PIN, INPUT);

  pinMode(TRANSMITTER_DISABLED_PIN, INPUT);  

// initialize the HD44780 LCD modulel
  pinMode(RS_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(D4_PIN, OUTPUT);
  pinMode(D5_PIN, OUTPUT);
  pinMode(D6_PIN, OUTPUT);
  pinMode(D7_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  
  lcd.begin(16,2);

// init the content matrix lcd_content[position]
  initArray(lcd_content, DISPLAY_SIZE, 32);
// init the SWR-array
  initArray(swr, BAND_COUNT, 10);
// init task array
  for(uint8_t i=0; i<=MID; i++) {
    tl[i] = -1;
  }

// initially turn off DDS (only to initialize the display)
  setPhaseValue(0, 0);
  pulsing_on = 0;

  digitalWrite(LED, HIGH);
// Display intro   
  writeToBuffer(INTRO);
  loadLCD();
  delay(2000);

/*
initializes the CRC library

Parameter:
 - the EEPROM start address to store the 256 byte CRC lookup table

 returns the next free address in EEPROM
*/
  uint16_t eeprom_address = 0;
  eeprom_address = CRC.init(eeprom_address);

// create datasets
  dataset[0] = DR.createDataset(0, 25, processDataset_0);
  dataset[1] = DR.createDataset(1, 35, processDataset_1);

/*
initializes the DataReceiver

Parameter DataReceiver (f.l.t.r.):
"BYTE_RATE" -> the data transfer rate set at the transmitter in Byte/s
"GPS_INPUT_PIN" -> the digital, interrupt-capable pin the receiver is connected to
"d_set" -> array of "dset", holding all dataset-relevant information
"DATASET_COUNT" -> the number of datasets (size of DATASET)
*/
  DR.init(BYTE_RATE, GPS_INPUT_PIN, dataset, 2);

// read beacon's band status and duty-cycle from EEPROM
  for(uint8_t i=0; i<BAND_COUNT; ++i) {
    band_status[i] = EEPROM.read(eeprom_address + i);
  }
  beacon_idle_level = min(EEPROM.read(eeprom_address + BAND_COUNT), 19);
    
// Check status of "disable transmitter" switch and if it's set to "off" start setup-dialog
  if(digitalRead(TRANSMITTER_DISABLED_PIN)) {
    Serial.begin(9600);
    writeToBuffer(SERIAL_DISP);
    loadLCD();
    do {
      Serial.println(F("\n\nWSPR beacon setup dialog.\n"));

      Serial.println(F("Current band settings:\n"));
      for(uint8_t i=0; i<BAND_COUNT; ++i) {
        printBand(i); Serial.print(F("m is "));
        if(band_status[i]) Serial.println(F("on"));
        else Serial.println(F("off"));
      }
      Serial.print(F("\nWould you like to change these settings (y/n)?"));
      if(readYesNo()) {
        Serial.println();
        for(uint8_t i=0; i<BAND_COUNT; ++i) {
          Serial.print(F("\nActivate ")); printBand(i); Serial.print(F("m (y/n)?"));
          if(readYesNo()) {
            band_status[i] = 1;
          }
          else {
            band_status[i] = 0;
          }
        }
      }

      Serial.println();
      printDutyCycle();
      Serial.print(F("\nTo increase/decrease (repeatedly) press \'i\' or \'d\', to quit press \'q\'."));
      int8_t dir;
      clearSerialInputBuffer();
      do {
        delay(100);
        dir = Serial.read();
        if((dir=='i' || dir=='I') && beacon_idle_level) {
          --beacon_idle_level;
          printDutyCycle();
        }
        else {
          if((dir=='d' || dir=='D') && beacon_idle_level < 19) {
            ++beacon_idle_level;
            printDutyCycle();
          }
        }
      }
      while(dir!='q' && dir!='Q');
    
      Serial.print(F("\n\nWould you like to discard changes and repeat setup? (y/n)?"));
    }
    while(readYesNo());
	
	  uint8_t i = 0;
	  while(i < BAND_COUNT) {
	    EEPROM.update(eeprom_address + i, band_status[i]);
	    ++i;
	  }
	  EEPROM.update(eeprom_address + i, beacon_idle_level);
    
    Serial.print(F("\n\nDone! Settings have been saved. You may now disconnect and restart the beacon."));
    endlessLoop();
  }
   
// Display band status
  const char DISABLED[3] = {'-', '-', '-'};
  const uint8_t POS[10] = {0, 4, 7, 10, 13, 17, 20, 23, 26, 29};
  uint8_t len = 3;
// BAND_INFO_FIXED_ELEMENTS
  writeToBuffer(BAND_INFO);
  uint8_t min_one_band_active = 0;
  for(uint8_t i=0; i<BAND_COUNT; i++) {
    if(!band_status[i]) {
      if(i) { len = 2; }
      writeCharArray(DISABLED, POS[i], len);
    }
    min_one_band_active += band_status[i];
  }
  loadLCD();
  delay(2000);

// test-code WSPR message & check band status (at least 1 must be active)
// in case of errors system will be halted
  if(!WSPR.encodeMessage(CALL, locator, POWER) || !min_one_band_active) {
    writeToBuffer(SETTINGS_NOT_VALID);
    loadLCD();
    endlessLoop();
  }

// Initially calculate and set the delta-phase values for WSPR lower band limits @ 160-10m
  for(uint8_t i=0; i<BAND_COUNT; i++) {
    deltaphase_base[i] = dds.calculatePhaseValue(pgm_read_dword_near(BASE_FREQUENCY + i));
  }

// initialize timer 2
  cli(); // disable global interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // set entire TCCR2B register to 0
  OCR2A = 59; // set compare match register to trigger every 30µs
  TCCR2A |= (1 << WGM21);  // turn on CTC mode
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
  sei(); // enable global interrupts

  setPhaseValue(0, 0);
  
// Schedule switching of transmitter
  scheduleTask(0, 0);

// call randomSeed()  
  scheduleTask(1, 0);
 
}

//##########################################################################################################
//##########################################################################################################

// timer 2 compa-isr (backlight pulsing)
ISR(TIMER2_COMPA_vect) {
  if(!backlight_on_time_counter) {
    digitalWrite(LED, 1);

    if(brightness==255) { brightness_change = -1; }
    else {
      if(brightness==MIN_BRIGHTNESS) {
        brightness_change = 1;
        scheduleTask(5, 0);
      }
    }

    brightness+=brightness_change;
  }
  else {
    if(backlight_on_time_counter == brightness) { digitalWrite(LED, 0); }
  }
  backlight_on_time_counter++;
}

//##########################################################################################################
//##########################################################################################################

void loop() {

// variables and constants used to calculate the loop - runtime
  static uint32_t stp = micros();
  static uint32_t st;
  static int32_t dt = 0;

  const int32_t MIN_DT = LONG_MIN>>1;

// trigger "process datasets" execution (runs every second)
  if(process_datasets_loop_counter == PROCESS_DATASETS_LOOPS) {

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // this makes sure the following block of code will not get interrupted 
      if(DR.getStatus()&4 && DR.validateData()) {//check if data is available and consistent and if so ...
        DR.setStatus(2); // ... mark dataset as "busy reading" and ...
        (dataset + DR.getPos())->FUNCTION(); //... process it by calling its function
        DR.setStatus(0); // release dataset to enable reception of next transmission
      }
    }

    // task has been executed -> set counter to 0
    process_datasets_loop_counter = 0;
  }

// trigger "transmit symbol" execution (runs every 683ms)
  if(transmit_symbol_loop_counter == TRANSMIT_SYMBOL_LOOPS) {
    if(on_air) {
      // terminate transmission after 110.6s
      if(symbol_counter == 162) {
        setPhaseValue(0, 0);
      }
      else {
        setPhaseValue(deltaphase + DELTAPHASE_PSK[WSPR.getSymbol(symbol_counter)], 0);
        symbol_counter++;
      }
    }

    transmit_symbol_loop_counter = 0;
  }

// trigger "system timer" execution (runs every 100ms)
  if(system_timer_loop_counter == SYSTEM_TIMER_LOOPS) {
    timer++;   
    system_timer_loop_counter = 0;
  }

// trigger "check SWR" execution (runs every 2.3ms)
  if(check_SWR_loop_counter == CHECK_SWR_LOOPS) {
  // readout SWR-meter and check SWR to be <= 3.0
    if(SWR_check_active) {
    // wait until forward reading has reached 25% of max-value to avoid false triggering
      fwd_value = analogRead(SWR_FWD_PIN);
      if(fwd_value > 255) {
        ref_value = getPolyValue(analogRead(SWR_REF_PIN), 18, SWR_X, SWR_Y);
        ref_value <<= 16;
        ref_value /= fwd_value;
        if(ref_value > 1712353) { ref_value = 1712353; }
        // store current swr-value in swr-averaging array
        swr_avg[swr_avg_pointer] = (20971520 + 10*ref_value)/(2097152 - ref_value);

        if(swr_avg[swr_avg_pointer] > 30) {
          setPhaseValue(0, 0);
          if(swr[band_pointer] > 30) {
            swr_avg[swr_avg_pointer] = 0;
          }
          swr[band_pointer] = swr_avg[swr_avg_pointer];
        }

        // increase swr-average array pointer
        swr_avg_pointer = (swr_avg_pointer == (SWR_AVG_LENGTH - 1)) ? 0 : swr_avg_pointer + 1;
      }
    }
  
    check_SWR_loop_counter = 0;
  }

/* trigger "single task scheduler" execution (runs every 11ms)

tasks can schedule other tasks or themselves from within this
routine; if a task reschedules itself, it becomes cyclical
until stopped externally by "scheduleTask(id, -1)"
*/
  if(single_task_scheduler_loop_counter == SINGLE_TASK_SCHEDULER_LOOPS) {
// scan array and execute due tasks
    for(uint8_t i=0; i<=MID; i++) {
      if(tl[i] > -1 && tl[i] <= timer) {
        tl[i] = -1;
          
        switch(i) {
// execute task 0 -> switching of transmitter
          case 0:
            // set "transmitter disabled" flag
            td = digitalRead(TRANSMITTER_DISABLED_PIN);

            scheduleTask(0, 1); // Re-schedule in 100ms
            
            // Shutdown WSPR transmitter and reset SWR-readings, if transmitter-disabled flag was set
            if(td) {
              if(!td_acknowledged) {
                setPhaseValue(0, 1);
                // init the SWR-array
                initArray(swr, BAND_COUNT, 10);
                td_acknowledged = 1;
              }
            }
            else {
              if(td_acknowledged) { td_acknowledged = 0; }
              /* Do some checks before initiating transmission sequence:
                Check that there is no ongoing transmission, that time is 0s into an even minute,
                that there is at least one band with an SWR < 3 and that the system time is valid
              */
              swr_sum = 0;
              for(uint8_t i=0; i<BAND_COUNT; i++) {
                if(band_status[i]) { swr_sum += swr[i]; }
              }
              if(!on_air && !second() && !(minute()%2) && swr_sum && gps_valid) {
				        // Check that the duty cycle is being met
				        if(!beacon_idle_counter) {
                  // delay transmission by 950ms (should actually start 1s into the even minute)
                  transmit_symbol_loop_counter = -267333/TLR;

                  // schedule SWR-readout to start 2.3ms after the beacon
                  if(SWR_METER_INSTALLED) {
                    check_SWR_loop_counter = -949800/TLR;
                    SWR_check_active = 1;
                    initArray(swr_avg, SWR_AVG_LENGTH, 10);
                    swr_avg_pointer = 0; 
                  }
                
                  // check if band is activated and SWR was ok during previous run
                  // if conditions are not met, roll-over
                  do {
                    band_pointer = (band_pointer == (BAND_COUNT - 1)) ? 0 : band_pointer + 1;
                  } while (!band_status[band_pointer] || !swr[band_pointer]);

                  // generate band information to be displayed on LCD
                  uint8_t offset = 0;
                  if(band_pointer > 4) { offset = 1; }
                  for(uint8_t i=0; i<3; i++) {
                    band[i] = pgm_read_byte_near(BAND_INFO + 3*band_pointer + offset + i);
                  }

                  deltaphase = deltaphase_base[band_pointer];
                  if(beacon_mode) {
                    // set a fixed transmit frequency
                    deltaphase += dds.calculatePhaseValue(beacon_mode);
                  }
                  else {
                    // set a random transmit frequency
                    deltaphase += random(0, DELTAPHASE_BANDWITH);
                  }

                  WSPR.encodeMessage(CALL, locator, POWER);
				
                  beacon_idle_counter = beacon_idle_level;

                  on_air = 1;
			          }
			        	else {
			        	  --beacon_idle_counter;
                  scheduleTask(0, 12); // Re-schedule in 1.2s
			        	}
              }
            }
          break;
// execute task 1 -> set new seed for the random number generator
          case 1:
            randomSeed(now());
            scheduleTask(1, 36000); // Re-schedule in 1h
          break;
// execute task 2 -> update those display items that require high-frequency refreshing
          case 2:
            switch(fast_refresh_pointer) {
              // refresh time & date
              case 0:
                static time_t ts_last = 0;
                time_t ts;
                ts = now();
                if(ts!=ts_last) {
                  ts_last = ts;

                  writeNumber(hour(ts), -3, 1, 2);
                  writeNumber(minute(ts), 0, 1, 2);
                  writeNumber(second(ts), 3, 1, 2);

                  uint8_t wd = 3*(weekday(ts)-1);
                  for(uint8_t i=0; i<3; i++) {
                    lcd_content[i + 16] = pgm_read_byte_near(WEEKDAY_CODING + i + wd);
                  }
          
                  writeNumber(day(ts), 18, 1, 2);
                  writeNumber(month(ts), 21, 1, 2);
                  writeNumber(year(ts), 26, 1, 4);

                  loadLCD();
                }
              break;
              // calculate and display average SWR
              case 1:
                // calculate swr-average
                static uint16_t swr_average;
                swr_average = 0;
                for(uint8_t i=0; i<SWR_AVG_LENGTH; i++) {
                  swr_average += swr_avg[i];
                }
                swr_average /= SWR_AVG_LENGTH;

                lcd_content[27] = '0'+swr_average/10;
                lcd_content[29] = '0'+swr_average%10;

                loadLCD();
                break;        
            }
            
            scheduleTask(2, 1); // Re-schedule in 100ms
          break;
// execute task 3 -> adjust system clock
          case 3:
            setTime(gps_time);
			last_sync = gps_time;
            gps_valid = 1;
          break;
// execute task 4 -> reschedule task 5
          case 4:
            scheduleTask(4, 35);
            scheduleTask(5, 0);
          break;
// execute task 5 -> change display content
          case 5:
            if(disp_content_pointer <= MAX_DISP_CONTENT) { setDisplayContent(); }
          break;
        }
      }
    }
    
    single_task_scheduler_loop_counter = 0;
  }
  
// increment all loop counters
  ++process_datasets_loop_counter;
  transmit_symbol_loop_counter++;
  system_timer_loop_counter++;
  single_task_scheduler_loop_counter++;
  check_SWR_loop_counter++;

// loop delay handling
// insures that on average the desired loop runtime will be met
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

//##########################################################################################################
// general purpose functions
// #########################################################################################################

// clears the Serial input buffer

void clearSerialInputBuffer() {
  while(Serial.available()) {
    Serial.read();
  }
}

//##########################################################################################################

// the point of no return :-)
void endlessLoop() {
  while(true) {
    digitalWrite(LED, LOW);
    delay(1000);
    digitalWrite(LED, HIGH);
    delay(1000);
  }
}    

//##########################################################################################################

// prints the band-value based on its position (0 = 160,...,9 = 10)
void printBand(uint8_t pos) {
  const uint8_t BANDS[BAND_COUNT] = {160,80,60,40,30,20,17,15,12,10};
  Serial.print(BANDS[pos]);
}
//##########################################################################################################

// prints the current duty cycle
void printDutyCycle() {
  Serial.print(F("\nThe beacon's duty cycle is 1/"));
  Serial.print(beacon_idle_level + 1); Serial.print(F(" = "));
  float percent = 100.0/(beacon_idle_level + 1);
  Serial.print(percent, 1); Serial.print(F("%."));
}

//##########################################################################################################

// Reads from PC's keyboard until YyNn has been pressed. Returns 1 for yes and 0 for no.
uint8_t readYesNo() {
  int16_t dummy = -1;
  clearSerialInputBuffer();
  while(dummy!='Y' && dummy!='y' && dummy!='N' && dummy!='n') {
    delay(100);
    dummy = Serial.read();
  }
  if(dummy=='Y' || dummy=='y') {
    Serial.print(F(" YES"));
    return 1;
  }
  else {
    Serial.print(F(" NO"));
    return 0;
  }
}

//##########################################################################################################

// support function / writes the Sun's and Moon's rise/set times in to the LCD-buffer
void writeRiseSet(uint8_t is_moon) {
	
  uint8_t start = 0;
  if(is_moon) start=2;

  time_t r_s, ts = now();
  for(uint8_t i=0; i<2; ++i) {
    uint8_t shift = i<<4;
    r_s = rs[start + i] + TIMEZONE*3600;
    writeNumber(hour(r_s), 8 + shift, 1, 2);
    writeNumber(minute(r_s), 11 + shift, 1, 2);
	if(day(ts)!=day(r_s) || month(ts)!=month(r_s) || year(ts)!=year(r_s)) { // is "today" or not?
	  lcd_content[7 + shift] = '.';
	  lcd_content[10 + shift] = '.';
      writeNumber(day(r_s), 2 + shift, 1, 2);
      writeNumber(month(r_s), 5 + shift, 1, 2);
    }
  }
}

//##########################################################################################################

// initializes an array of the type uint8_t with the same number
void initArray(uint8_t *target, uint8_t target_size, uint8_t value) {
  for(uint8_t i=0; i<target_size; i++) {
    *(target+i) = value;
  }
}

//##########################################################################################################

// calculate and returns the corresponding y to a given x based on a linear polynomial fit / returns 0 if x
// yields no result
// "x" -> input value
// "size" -> reference array size
// array_x & array_y -> reference arrays

uint16_t getPolyValue(const int16_t x, const uint16_t size, const uint16_t* array_x, const uint16_t* array_y) {

  uint16_t y = 0;
  uint16_t xbegin, xend, ybegin, yend;

// search for the supporting points enclosing x and if found, calculate y
  for(uint16_t i=0; i<size-1; i++) {
  
    xbegin = pgm_read_dword_near(array_x + i);
    xend = pgm_read_dword_near(array_x + i + 1);

    if((xbegin<=xend && x>=xbegin && x<=xend) || (xbegin>xend && x<=xbegin && x>=xend)) {

    ybegin = pgm_read_dword_near(array_y + i);
    yend = pgm_read_dword_near(array_y + i + 1);

      y = ybegin + int32_t((yend-ybegin)*(x-xbegin))/(xend-xbegin);
      break;
    }
  }

  return y;
}

//##########################################################################################################
// functions used for backlight control
//##########################################################################################################

// turns backlight pulsing on/off
void backlightPulsingOn(uint8_t pulsing) {
  if(pulsing) {
    if(!pulsing_on) {
      // initialize timer 2
      scheduleTask(4, -1);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        brightness = 255;
        backlight_on_time_counter = 0;
        TCNT2  = 0; //initialize counter value to 0
        TCCR2B = 0b00000010; // prescaler 8
      }

      pulsing_on = 1;
    }
  }
  else {
    if(pulsing_on) {
      // disable timer 2
      scheduleTask(4, 35);
      TCCR2B = 0; // set entire TCCR2B register to 0
      
      digitalWrite(LED, 1);
      pulsing_on = 0;
    }
  }
}

//##########################################################################################################
// functions used for HD44780 control
//##########################################################################################################

// Sets the display content and updates the LCD; if short cycle updating is needed, a rapid update task will
// be started automatically
void setDisplayContent() {

// unschedule rapid updates  
  scheduleTask(2, -1);

// clear display buffer
  initArray(lcd_content, DISPLAY_SIZE, 32);

// common part -> display transmitter status
  if(!disp_content_pointer) {
    if(!pulsing_on) {
      writeToBuffer(TX_ON_AIR);
      writeCharArray(band, 16, 3);
      if(SWR_METER_INSTALLED) {
        // slow down content switching when SWR is being displayed
        scheduleTask(4, 140);
        fast_refresh_pointer = 1;
        scheduleTask(2, 0);
      }
    }
    else {
	  // the maximum permissive age [seconds] of the last valid dataset before transmission will be suspended
      if(last_sync && (now()-last_sync) > 300) {
        writeToBuffer(TX_GPS_DISABLED);
        gps_valid = 0;
      }
      else {
        if(td) {
		  writeToBuffer(TX_HARDWARE_DISABLED);
        }
        else {
          if(!swr_sum) {
            writeToBuffer(TX_SWR_DISABLED);
            lcd_content[25] = 32;
            lcd_content[26] = 32;
            lcd_content[31] = 32;
          }
          else {
            if(swr[band_pointer]>30) {
              writeToBuffer(TX_SWR_DISABLED);
              writeCharArray(band, 28, 3);
            }
            else {
              writeToBuffer(TX_STANDBY);
            }
          }
        }
      }
    }
  }
  else {
    if(gps_valid) {
      switch(disp_content_pointer) {
        case 1:// Display user settings   
          writeToBuffer(USER_DATA);
          writeCharArray(CALL, 0, 6);
          writeCharArray(locator, 7, 4);
          writeCharArray(loc, 12, 2);
          writeNumber(POWER, 21, 0, 4);
        break;
        case 2:// display time, date & temp
          // LCD_TIME_FIXED_ELEMENTS
          writeToBuffer(TIME_DATE);
          lcd_content[13] = 223;
          // write temperature
          writeNumber(temp, 8, 0, 3);
          if(TEMP_SCALE) { lcd_content[14] = 'F'; }
          // schedule rapid update of time & date
          fast_refresh_pointer = 0;
          scheduleTask(2, 0);
        break;
        case 3:
          if(COORDINATES) { // display longitude & latitude
            uint8_t shift;
            for(uint8_t i=0; i<2; ++i) {
              shift = i*16;
              lcd_content[3 + shift] = 223;
              lcd_content[7 + shift] = '\'';
              lcd_content[11 + shift] = '\"';
              lcd_content[13 + shift] = lat_long[3][i];
              for(uint8_t j=0; j<3; ++j) {
                writeNumber(lat_long[j][i], -2 + (j<<2) + shift, 1, 2);
              }
            }
          }
          else {
            scheduleTask(5, 0);
          }
        break;
        case 4:
          if(ALT_SPEED) { // display altitude and speed over ground
            // LCD_ALT_SPEED_FIXED_ELEMENTS
            writeToBuffer(ALT_SPEED_DISP);
            if(DIST_UNIT) { // feet instead of meter
              char ft[3] = "ft";
              writeCharArray(ft, 14, 2);
            }
            writeNumber(alt, 9, 0, 4);

            if(SPEED_UNIT) {
              lcd_content[28] = 32;
              lcd_content[29] = 32;
              if(SPEED_UNIT & 1) { // mph instead of km/h
                char mph[4] = "mph";
                writeCharArray(mph, 26, 3);
              }
              else { // knots instead of km/h
                lcd_content[27] = 'n';
              }
            }
            writeNumber(sog, 21, 0, 3);
          }
          else {
            scheduleTask(5, 0);
          }
        break;
        case 5:
          if(SUN && rs[0]) { // display Sun's set- and rise-time
            // LCD_SUN_RISE_SET_FIXED_ELEMENTS
            writeToBuffer(SUN_DISP);
            writeRiseSet(0);
          }
          else {
            scheduleTask(5, 0);
          }
        break;
        case 6:
          if(MOON && rs[0]) { // display Moon's set- and rise-time
            // LCD_MOON_RISE_SET_FIXED_ELEMENTS
            writeToBuffer(MOON_DISP);
            writeRiseSet(1);
          }
          else {
            disp_content_pointer = MAX_DISP_CONTENT;
            scheduleTask(5, 0);
          }
        break;
        case 7: // display Moon's phase
          // LCD_MOON_RISE_SET_FIXED_ELEMENTS
          writeToBuffer(MOON_PHASE_DISP);
          writeNumber((abs(phase)+50)/100, 15, 0, 3);
          if(phase > 0) {
            char in[3] = "in";
            writeCharArray(in, 23, 2);
          }
        break;
      }
    }
    else {
      // LCD_WAITING_FOR_GPS_FIXED_ELEMENTS
      writeToBuffer(WAITING_FOR_GPS);
      disp_content_pointer = MAX_DISP_CONTENT;
    }
  }

// increment the "display content pointer"
  disp_content_pointer = (disp_content_pointer == MAX_DISP_CONTENT) ? 0 : disp_content_pointer + 1;
  
  loadLCD();
}

//##########################################################################################################

// Updates the LCD content (only values that have changed will be transferred)
void loadLCD() {
  uint8_t c_pos = cursor_position;
  uint8_t c_stat = cursor_status;
  
  for(uint8_t i=0; i<DISPLAY_SIZE; i++) {
    if(lcd_content_last[i] != lcd_content[i]) {
      placeCursor(i, 0);
      lcd.write(lcd_content[i]);
      lcd_content_last[i] = lcd_content[i];
    }
  }
  
  placeCursor(c_pos, c_stat);
}

//##########################################################################################################

// Sets the cursor to a given position (0...31)
// with a given status (0==nothing; 1==only corsor; 2==only blinking; 3==blinking cursor)
void placeCursor(uint8_t c_pos, uint8_t c_stat) {
  cursor_position = c_pos;
  cursor_status = c_stat;
  
  uint8_t row = c_pos>>4;
  if(c_pos > 15) {
    c_pos -= 16;
  }
  lcd.setCursor(c_pos, row);

  switch(c_stat) {
    case 0:
      lcd.noCursor();
      lcd.noBlink();
    break;
    case 1:
      lcd.cursor();
      lcd.noBlink();
    break;
    case 2:
      lcd.noCursor();
      lcd.blink();
    break;
    case 3:
      lcd.cursor();
      lcd.blink();
    break;
    default:
      lcd.noCursor();
      lcd.noBlink();
  }
}

//##########################################################################################################

// Writes a full set of 32 characters from an array to the LCD screen buffer
void writeToBuffer(const uint8_t* array) {
  for(uint8_t i=0; i<DISPLAY_SIZE; i++) {
    lcd_content[i] = pgm_read_byte_near(array + i);
  }
}

//##########################################################################################################

// Writes a 4 digit number into consecutive LCD-segments; the number of leading zeros can be specified
// spanning from 0 to 3; the max. number of digits can be specified spanning from 1 to 4; neg. values will be
// preceeded by "-"
void writeNumber(int16_t number, int8_t pos, uint8_t leading_zeros, uint8_t max_digits) {
  char text[6] = " 0000";
  if(number < 0) {
    number = abs(number);
    text[0] = '-';
  }

  uint16_t dummy;
  uint8_t e = 1;
  int8_t sp = 1;
  uint8_t ns = 1;
  for(uint16_t i=1000; i>1; i/=10) {
    dummy = number/i;
    if(dummy) {
      text[e] += dummy;
      ns = 0;
    }
    else {
      if(ns) { sp++; }
    }
    number %= i;
    e++;
  }
  text[4] += number;
  
  sp -= leading_zeros;
  if(sp < 1) { sp = 1; }
  
  uint8_t ssize = 5-max_digits;
  if(ssize > sp) { sp = ssize; }
  
  if(text[0]=='-') {
    sp--;
    text[sp] = '-';
  }

  writeCharArray(&text[sp], pos+sp, 5-sp);
}

//##########################################################################################################

// Writes a set of characters to LCD screen buffer
void writeCharArray(const char* char_array, uint8_t pos, uint8_t len) {
  for(uint8_t i=0; i<len; i++) {
    lcd_content[i + pos] = *(char_array + i);
  }
}

//##########################################################################################################
// functions used for AD9850 control
//##########################################################################################################

// sets phase and restarts DDS
// if shutdown is 1, DDS will shut down, reducing the dissipated power from 380mW to 30mW @5V
void setPhaseValue(uint32_t deltaphase, uint8_t shutdown) {
  dds.setPhase(deltaphase, 0, shutdown);
	
  if(deltaphase) {
    backlightPulsingOn(0);
  }
  else {
    SWR_check_active = 0;
    symbol_counter = 0;
    on_air = 0;
    backlightPulsingOn(1);
    disp_content_pointer = 0;
    setDisplayContent();
  }

}

//##########################################################################################################
// functions used for task scheduling
//##########################################################################################################

// schedules a new task for single execution or unschedules a task
// "delay" should be given in 100ms resolution (10 == 1s; 0 == immediate execution; <0 == off/unschedule)

void scheduleTask(uint8_t id, int32_t delay) {
// check validity of task-id
  if(id <= MID) {

// schedule task by entering delay time into task-list
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if(delay > -1) {
        tl[id] = timer + delay;
      }
      else {
        tl[id] = -1;
      }
    }
  }
}
