/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
November  2013     V2.3
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

//void interruptRisingFunction();
volatile uint64_t interrupt_counter = 0;//MDP -interrupt debug counter
volatile uint16_t heightMeas = 0;//unknown units

//===================Debug Variables================================
unsigned long dbg_sensor_read_time = 0;
uint16_t dbgStartTime = 0;
int16_t  debug[10] = {0,0,0,0,0,0,0,0,0,0};
bool dbg_toggle = true;
//==================================================================

//==================Attitude Controller Variables (MDP)===================================
int16_t prevAttError[3] = {0,0,0}; //Used for calculating the derivative of the error
int8_t userControlFlag = 0; //0 means autonomous, 1 means user can control roll and pitch with old multii controller + gains
//========================================================================================

//=====================IR Sensor Variables (MDP)==================================
volatile bool currentState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; //For Sonar
volatile bool previousState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH,HIGH}; //For Sonar

volatile int16_t fallTime[6] = {0, 0, 0, 0, 0,0};
volatile int16_t elapsedTime[6] = {0, 0, 0, 0, 0, 0};
volatile int16_t elapsedTimePrevious[6] =  {0, 0, 0, 0, 0, 0};
volatile uint8_t pinArray[5] = {7, 5, 4, 2, 6};
volatile bool sensorUpdate = false;
volatile uint16_t TimeoutCounter[6] = {0,0,0,0,0,0}; //counter to determine if a sensor is not receiving new data
volatile bool SensorTimeoutFlag[6] = {HIGH,HIGH,HIGH,HIGH,HIGH,HIGH}; //flag which indicates a given sensor is not receiving new data
//================================================================================

//Distance variables
int32_t SUM;
int32_t theta_x, theta_y, theta_z;
int32_t dx, dy;
int32_t Rxy;
int32_t Rz;
int32_t Sin_x, Sin_y, Cos_z;
float x, y, z;
int16_t yawIR = 0;
float theta = 0.0f;
bool quad_behind = true; //TFC - used for determing quad heading for landing
float quad_mass = 0.096; //TFC - [kg]

//MDP - LPF Variables
float rawDataX[2] = {0};
float rawDataY[2] = {0};
float rawDataRxy[2] = {0};
float rawDataRz[2] = {0};
float filteredDataX[2] = {0};
float filteredDataY[2] = {0};
float filteredDataRxy[2] = {0};
float filteredDataRz[2] = {0};
float filteredAdjustRxy[2] = {0};

float filteredYaw[2] = {0};

uint8_t filter_counter = 0;//MDP - is this even used?

//========================Behavior Variables========================
bool hitFlag = LOW;
bool landFlag = LOW;    //TFC
bool forwardFlag = LOW; //TFC
bool flipFlag = LOW;
bool keepdistFlag = LOW;
bool chasecarFlag = LOW;
bool swingFlag = LOW;
bool danceYAWflag = LOW;
int16_t desiredDanceYaw = 1500;
//==================================================================

/*********** RC alias *****************/

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
  ;

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
#if ACC
  "ANGLE;"
  "HORIZON;"
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  "BARO;"
#endif
#ifdef VARIOMETER
  "VARIO;"
#endif

#ifdef HEADFREE
  "HEADFREE;" // 5
#endif
#ifdef HEADHOLD
  "HEADHOLD;" // 6
#endif

#if MAG
  "MAG;"
  "HEADFREE;"
  "HEADADJ;"
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  "PASSTHRU;"
#endif
#if defined(BUZZER)
  "BEEPER;"
#endif
#if defined(LED_FLASHER)
  "LEDMAX;"
  "LEDLOW;"
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  "CALIB;"
#endif
#ifdef PID_SWITCH
  "PID SW;"
#endif
#ifdef OSD_SWITCH
  "OSD SW;"
#endif
  ;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
#if ACC
  1, //"ANGLE;"
  2, //"HORIZON;"
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  3, //"BARO;"
#endif
#ifdef VARIOMETER
  4, //"VARIO;"
#endif
#if MAG
  5, //"MAG;"
  6, //"HEADFREE;"
  7, //"HEADADJ;"
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
  8, //"CAMSTAB;"
#endif
#if defined(CAMTRIG)
  9, //"CAMTRIG;"
#endif
#if GPS
  10, //"GPS HOME;"
  11, //"GPS HOLD;"
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  12, //"PASSTHRU;"
#endif
#if defined(BUZZER)
  13, //"BEEPER;"
#endif
#if defined(LED_FLASHER)
  14, //"LEDMAX;"
  15, //"LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  16, //"LLIGHTS;"
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  17, //"CALIB;"
#endif
#ifdef PID_SWITCH
  18, //"pid_sw;"
#endif
#ifdef OSD_SWITCH
  19, //"OSD_SWITCH;"
#endif
};

//MDP - used to run position control and measurements at a fraction of the rate of the 50Hz loop
float roll_cmd = 0;
float pitch_cmd = 0;
float thrust_cmd = 0;
uint8_t pos_controller_counter = 0;

int16_t desiredHeight = 100;
float desiredX = 0;
float desiredY = 0;
float desiredYaw = -90;

int16_t sonarDistance = 0;

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold, headFreeModeHold; // [-180;+180]
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0, 0, 0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

#if defined(ARMEDTIMEWARNING)
uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

flags_struct_t f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
uint16_t cycleTimeMax = 0;       // highest ever cycle timen
uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
int32_t  BAROaltMax;             // maximum value
uint16_t GPS_speedMax = 0;       // maximum speed from gps
uint16_t powerValueMaxMAH = 0;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
uint32_t armedTime = 0;
#endif

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;



#if defined(THROTTLE_ANGLE_CORRECTION)
int16_t throttleAngleCorrection = 0;	// correction of throttle in lateral wind,
int8_t  cosZ = 100;					// cos(angleZ)*100
#endif



// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
uint16_t InflightcalibratingA = 0;
int16_t AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
uint16_t powerValue = 0;          // last known current
#endif
uint16_t intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
uint8_t telemetry = 0;
uint8_t telemetry_auto = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
uint8_t telemetryStepIndex = 0;
#endif

// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcDataCmds[RC_CHANS];// interval [1000:2000], MDP - bypasses modifications multiwii does to rcData[]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SPEKTRUM)
volatile uint8_t  spekFrameFlags;
volatile uint32_t spekTimeLast;
#endif

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[8];
int16_t servo[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1000};

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

#ifdef LOG_PERMANENT
plog_t plog;
#endif

// **********************
// GPS common variables
// **********************
int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
int16_t  GPS_directionToHome;                         // direction to home - unit: degree
uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
uint8_t  GPS_Enable  = 0;

// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
int16_t  nav[2];
int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

uint8_t alarmArray[16];           // array

#if BARO
int32_t baroPressure;
int32_t baroTemperature;
int32_t baroPressureSum;
#endif

void annexCode() 
{ // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[THROTTLE] > 1500) 
  { // breakpoint is fix: 1500
    if (rcData[THROTTLE] < 2000) 
    {
      prop2 -=  ((uint16_t)conf.dynThrPID * (rcData[THROTTLE] - 1500) >> 9); //  /512 instead of /500
    } 
    else 
    {
      prop2 -=  conf.dynThrPID;
    }
  }

  for (axis = 0; axis < 3; axis++) 
  {
    //MDP - overwrite roll and pitch from position controller - Can we remove this?

    if (!userControlFlag)
    {
      if (axis == 0)
      {
        //rcData[axis] = (roll_cmd*10.0f) + 1500;
      }
      else if (axis == 1)
      {
        //rcData[axis] = (pitch_cmd*10.0f) + 1500;
      }
    }
    
    
    tmp = min(abs(rcData[axis] - MIDRC), 500);
#if defined(DEADBAND)
    if (tmp > DEADBAND)
    {
      tmp -= DEADBAND;
    }
    else 
    {
      tmp = 0;
    }
#endif
    if (axis != 2) //ROLL & PITCH
    { 
      tmp2 = tmp >> 7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp - (tmp2 << 7)) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) >> 7);
      prop1 = 128 - ((uint16_t)conf.rollPitchRate * tmp >> 9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1 * prop2 >> 7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8 * prop1 >> 7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8 * prop1 >> 7; // was /100, is /128 now
    } 
    else
    { // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  tmp = (uint32_t)(tmp - MINCHECK) * 2559 / (2000 - MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp / 256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 256) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  if (f.HEADFREE_MODE) { //to optimize
#if defined(HEADFREE) || defined(HEADHOLD)
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f;
#else
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
#endif
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    
    int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader = 0;
  switch (analogReader++ % 3) {
#if defined(POWERMETER_HARD)
    case 0:
      {
        uint16_t pMeterRaw; // used for current reading
        static uint32_t lastRead = currentTime;
        static uint8_t ind = 0;
        static uint16_t pvec[PSENSOR_SMOOTH], psum;
        uint16_t p =  analogRead(PSENSORPIN);
        //LCDprintInt16(p); LCDcrlf();
#if PSENSOR_SMOOTH != 1
        psum += p;
        psum -= pvec[ind];
        pvec[ind++] = p;
        ind %= PSENSOR_SMOOTH;
        p = psum / PSENSOR_SMOOTH;
#endif
        powerValue = ( conf.psensornull > p ? conf.psensornull - p : p - conf.psensornull); // do not use abs(), it would induce implicit cast to uint and overrun
        analog.amperage = powerValue * conf.pint2ma;
        pMeter[PMOTOR_SUM] += ((currentTime - lastRead) * (uint32_t)((uint32_t)powerValue * conf.pint2ma)) / 100000; // [10 mA * msec]
        lastRead = currentTime;
        break;
      }
#endif // POWERMETER_HARD

#if defined(VBAT)
    case 1:
      {
        static uint8_t ind = 0;
        static uint16_t vvec[VBAT_SMOOTH], vsum;
        uint16_t v = analogRead(V_BATPIN);
#if VBAT_SMOOTH == 1
        analog.vbat = (v << 4) / 265; //conf.vbatscale; // result is Vbatt in 0.1V steps
#else
        vsum += v;
        vsum -= vvec[ind];
        vvec[ind++] = v;
        ind %= VBAT_SMOOTH;
#if VBAT_SMOOTH == 16
        analog.vbat = vsum / conf.vbatscale; // result is Vbatt in 0.1V steps
#elif VBAT_SMOOTH < 16
        analog.vbat =  (vsum * (16 / VBAT_SMOOTH)) / conf.vbatscale; // result is Vbatt in 0.1V steps
#else
        analog.vbat = ((vsum / VBAT_SMOOTH) * 16) / conf.vbatscale; // result is Vbatt in 0.1V steps
#endif
#endif

        break;
      }
#endif // VBAT
#if defined(RX_RSSI)
    case 2:
      {
        static uint8_t ind = 0;
        static uint16_t rvec[RSSI_SMOOTH], rsum;
        uint16_t r = analogRead(RX_RSSI_PIN);
#if RSSI_SMOOTH == 1
        analog.rssi = r;
#else
        rsum += r;
        rsum -= rvec[ind];
        rvec[ind++] = r;
        ind %= RSSI_SMOOTH;
        r = rsum / RSSI_SMOOTH;
        analog.rssi = r;
#endif
        break;
      }
#endif
  } // end of switch()

  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {
      LEDPIN_OFF;
    }
    if (f.ARMED)
    {
#if defined(VBAT)
      if ((analog.vbat > conf.vbatlevel_warn1)) {
        LEDPIN_ON;
      }
      else {
        static unsigned long lblink = millis() + 333;
        if (millis() >= lblink) {
          LEDPIN_TOGGLE;
          lblink = millis() + 333;
        }
      }
#else
      LEDPIN_ON;
#endif
    }
  }


#if defined(LED_FLASHER)
  auto_switch_led_flasher();
#endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

#if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
#if defined(GPS_PROMINI)
  if (GPS_Enable == 0) {
    serialCom();
  }
#else
  //if (pos_controller_counter == 1) //MDP
    serialCom();//Runs at 4-6ms
#endif
#endif

#if defined(POWERMETER)
  analog.intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
  intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE;
#endif

#ifdef LCD_TELEMETRY_AUTO
  static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
  static uint8_t telemetryAutoIndex = 0;
  static uint16_t telemetryAutoTimer = 0;
  if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ) {
    telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
    LCDclear(); // make sure to clear away remnants
  }
#endif
#ifdef LCD_TELEMETRY
  static uint16_t telemetryTimer = 0;
  if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
#if (LCD_TELEMETRY_DEBUG+0 > 0)
    telemetry = LCD_TELEMETRY_DEBUG;
#endif
    if (telemetry) lcd_telemetry();
  }
#endif

#if GPS & defined(GPS_LED_INDICATOR)       // modified by MIS to use STABLEPIN LED for number of sattelites indication
  static uint32_t GPSLEDTime;              // - No GPS FIX -> LED blink at speed of incoming GPS frames
  static uint8_t blcnt;                    // - Fix and sat no. bellow 5 -> LED off
  if (currentTime > GPSLEDTime) {          // - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ...
    if (f.GPS_FIX && GPS_numSat >= 5) {
      if (++blcnt > 2 * GPS_numSat) blcnt = 0;
      GPSLEDTime = currentTime + 150000;
      if (blcnt >= 10 && ((blcnt % 2) == 0)) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
    } else {
      if ((GPS_update == 1) && !f.GPS_FIX) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
      blcnt = 0;
    }
  }
#endif

#if defined(LOG_VALUES) && (LOG_VALUES >= 2)
  if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
  if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
#endif
  if (f.ARMED)  {
#if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
    armedTime += (uint32_t)cycleTime;
#endif
#if defined(VBAT)
    if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
#endif
#ifdef LCD_TELEMETRY
#if BARO
    if ( (alt.EstAlt > BAROaltMax) ) BAROaltMax = alt.EstAlt;
#endif
#if GPS
    if ( (GPS_speed > GPS_speedMax) ) GPS_speedMax = GPS_speed;
#endif
#endif
  }
}

static uint32_t RCTime  = 0;
static uint32_t PIDTime = 0;
volatile uint16_t startTime = 0;

//**** Function Prototypes **** //TFC
void go_disarm();
void go_arm();

void setup()
{

  
  
#if !defined(GPS_PROMINI)
  SerialOpen(0, SERIAL0_COM_SPEED);
#if defined(PROMICRO)
  SerialOpen(1, SERIAL1_COM_SPEED);
#endif
#if defined(MEGA)
  SerialOpen(1, SERIAL1_COM_SPEED);
  SerialOpen(2, SERIAL2_COM_SPEED);
  SerialOpen(3, SERIAL3_COM_SPEED);
#endif
#endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readGlobalSet();
#ifndef NO_FLASH_CHECK
#if defined(MEGA)
  uint16_t i = 65000;                             // only first ~64K for mega board due to pgm_read_byte limitation
#else
  uint16_t i = 32000;
#endif
  uint16_t flashsum = 0;
  uint8_t pbyt;
  while (i--) {
    pbyt =  pgm_read_byte(i);        // calculate flash checksum
    flashsum += pbyt;
    flashsum ^= (pbyt << 8);
  }
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  global_conf.currentSet = 2;
#else
  global_conf.currentSet = 0;
#endif
  while (1) {                                                   // check settings integrity
#ifndef NO_FLASH_CHECK
    if (readEEPROM()) {                                         // check current setting integrity
      if (flashsum != global_conf.flashsum) update_constants(); // update constants if firmware is changed and integrity is OK
    }
#else
    readEEPROM();                                               // check current setting integrity
#endif
    if (global_conf.currentSet == 0) break;                     // all checks is done
    global_conf.currentSet--;                                   // next setting for check
  }
  readGlobalSet();                              // reload global settings for get last profile number
#ifndef NO_FLASH_CHECK
  if (flashsum != global_conf.flashsum) {
    global_conf.flashsum = flashsum;          // new flash sum
    writeGlobalSet(1);                        // update flash sum in global config
  }
#endif
  readEEPROM();                                 // load setting data from last used profile
  blinkLED(2, 40, global_conf.currentSet + 1);
  configureReceiver();
#if defined (PILOTLAMP)
  PL_INIT;
#endif
  initSensors();
#if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)
  GPS_set_pids();
#endif
  previousTime = micros();
#if defined(GIMBAL)
  calibratingA = 512;
#endif
  calibratingG = 512 + 256;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
#if defined(POWERMETER)
  for (uint8_t j = 0; j <= PMOTOR_SUM; j++) pMeter[j] = 0;
#endif
  /************************************/
#if defined(GPS_SERIAL)
  GPS_SerialInit();
  for (uint8_t j = 0; j <= 5; j++) {
    GPS_NewData();
    LEDPIN_ON
    delay(20);
    LEDPIN_OFF
    delay(80);
  }
  if (!GPS_Present) {
    SerialEnd(GPS_SERIAL);
    SerialOpen(0, SERIAL0_COM_SPEED);
  }
#if !defined(GPS_PROMINI)
  GPS_Present = 1;
#endif
  GPS_Enable = GPS_Present;
#endif
  /************************************/

#if defined(I2C_GPS) || defined(GPS_FROM_OSD)
  GPS_Enable = 1;
#endif

#if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) || defined(OLED_DIGOLE) || defined(LCD_TELEMETRY_STEP)
  initLCD();
#endif
#ifdef LCD_TELEMETRY_DEBUG
  telemetry_auto = 1;
#endif
#ifdef LCD_CONF_DEBUG
  configurationLoop();
#endif
#ifdef LANDING_LIGHTS_DDR
  init_landing_lights();
#endif
#ifdef FASTER_ANALOG_READS
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
#endif
#if defined(LED_FLASHER)
  init_led_flasher();
  led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
#endif
  f.SMALL_ANGLES_25 = 1; // important for gyro only conf
#ifdef LOG_PERMANENT
  // read last stored set
  readPLog();
  plog.lifetime += plog.armed_time / 1000000;
  plog.start++;         // #powercycle/reset/initialize events
  // dump plog data to terminal
#ifdef LOG_PERMANENT_SHOW_AT_STARTUP
  dumpPLog(0);
#endif
  plog.armed_time = 0;   // lifetime in seconds
  //plog.running = 0;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
#endif

  currentTime = micros();
  RCTime = currentTime;
  PIDTime = currentTime;
  debugmsg_append_str("initialization completed\n");

  //mwm: adding Davin's sonar hack
  //pinMode(4, INPUT);
  //enableInterrupt(4, interruptFallingFunction, FALLING);

  //MDP - IR sensors
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT);  //pullup ???
  pinMode(16, INPUT);//sonar
  pinMode(12, INPUT); // Port PB4 as input for Sonar
  pinMode(15, OUTPUT);
  //att.heading = -180;
  filteredYaw[0] = 0;

}

ISR(TIMER2_OVF_vect)
{
  sensorUpdate = true;
  
 #ifdef WS_QUADX
  currentState[0] = PIND & _BV(PD5);
  currentState[1] = PIND & _BV(PD2);
  currentState[2] = PIND & _BV(PD4);
  currentState[3] = PIND & _BV(PD6);
  currentState[4] = PIND & _BV(PD7);
#endif
  
#ifdef EL_QUAD  //EL
  currentState[0] = PIND & _BV(PD6);
  currentState[1] = PIND & _BV(PD4);
  currentState[2] = PIND & _BV(PD5);
  currentState[3] = PIND & _BV(PD7);
  currentState[4] = PIND & _BV(PD2);
#endif  
  
  //currentState[5] = PINC & _BV(2);//Sonar on PC2 (Quad#1)
  currentState[5] = PINB & _BV(4);//Sonar on AVR pin PB4
  
  for (int i = 0; i <= 5; i++)
  {
    if (!(currentState[i] == previousState[i]))
    {
      SensorTimeoutFlag[i] = LOW; //we saw a transition therefore beacon is on
      if (currentState[i] == HIGH)
      {
        elapsedTime[i] = micros() - fallTime[i];
      }
      else
      {
        fallTime[i] = micros();
      }
    }
	
    previousState[i] = currentState[i];
  }
}


void GetPhiAngleFromIR()
{
	int16_t Phi;
	static int16_t sensorData[4], prevsensorData[4];
	int16_t maxSensorData = 0;
	int8_t maxSensorId = 0, secondmaxSensorId = 0;
	int16_t secondmaxSensorData = 0;
	int16_t trigData[4] = {0,0,0,0};
	
// 	for (int8_t i = 0; i < 4; i++)
// 	{
// 		//implementing filter for peripheral IR sensors with alpha = 0.17
// 		sensorData[i] = (int16_t)((1.0-0.17)*(float)prevsensorData[i] + (0.17)*(float)elapsedTime[i]);
// 		prevsensorData[i] = sensorData[i];
// 		
// 		if(sensorData[i] > maxSensorData)
// 		{
// 			secondmaxSensorData = maxSensorData;
// 			secondmaxSensorId = maxSensorId;
// 			maxSensorData = sensorData[i];
// 			maxSensorId = i;
// 		}
// 		else if(sensorData[i] > secondmaxSensorData)
// 		{
// 			secondmaxSensorData = sensorData[i];
// 			secondmaxSensorId = i;
// 		}
// 	}
// 	
// 	trigData[maxSensorId] =  maxSensorData;
// 	trigData[secondmaxSensorId] = secondmaxSensorData;
// 	
	for (int8_t i = 0; i < 4; i++)
	{
		sensorData[i] = elapsedTime[i];
	}
	dx = sensorData[0] + sensorData[1] - sensorData[2] - sensorData[3];
	dy = sensorData[0] + sensorData[3] - sensorData[2] - sensorData[1];
	int32_t SUM4 = sensorData[0] + sensorData[3] + sensorData[2] + sensorData[1];
	
	if(SUM4 == 0) SUM4 = 1;
	dx = (dx << 7);
	dx = dx / SUM4;
	dy = (dy << 7);
	dy = dy / SUM4;	
	
	//Phi = _atan2(DX , DY) / 10;
}

void GetDistanceFromIR()
{
	int16_t distance;
	static int16_t bottomIRdata, prevbottomIRdata;
	//filtering bottom IR data with alpha = 0.17
	bottomIRdata = (int16_t)((1.0-0.17)*(float)prevbottomIRdata + (0.17)*(float)elapsedTime[4]);
	prevbottomIRdata = bottomIRdata;
	//converting IR sensor 
	distance = (6400 -  bottomIRdata)/24;
   
	if (distance < 0) distance = 0;
	debug[0] = distance; //prefiltered RXY
}
int16_t GetZenithFromIR()
{
	int16_t sensorData[5];
	int16_t maxSensorData = 0;
	int8_t maxSensorIdx = 0;
	int8_t leftIdx, rightIdx;
	int32_t Horiz, Vert, Zenith;
	double fZenith;
	int16_t Theta;
	
  for (int8_t i = 0; i <= 4; i++)
  { 
	  sensorData[i] = elapsedTime[i];
	  
	  if (sensorData[i] > maxSensorData)
	  {
		  if (i < 4)//Exclude the bottom sensor from this
		  {
			  maxSensorData = sensorData[i];
			  maxSensorIdx = i;
		  }
	  }
	  
	  sensorData[4] = elapsedTime[4];
    // TFC - get quad orientation
	  if (maxSensorIdx == 0) //check edge case
	  {
		  leftIdx = 3;
		  rightIdx = 1;
      quad_behind = true;
	  }
	  else if (maxSensorIdx == 3) //check edge case
	  {
		  leftIdx = 2;
		  rightIdx = 0;
      quad_behind = false;
	  }
	  else //Otherwise it is the neighbouring indices
	  {
		  leftIdx = maxSensorIdx - 1;
		  rightIdx = maxSensorIdx + 1;
      if(maxSensorIdx == 0)
      {
        quad_behind = true;
      }
      else
      {
        quad_behind = false;
      }
      
	  }

  }
  
	  Horiz = (2*sensorData[maxSensorIdx] + sensorData[leftIdx] + sensorData[rightIdx]);
	  Vert = 2*sensorData[4]; //should be 4 but we decrease to 2 since the peripheral sensors are weaker.
	  Zenith = (Horiz);
	  Zenith = Zenith * 900;
	  SUM = (Horiz + Vert);
	  if(SUM==0) SUM = 1;
	  Zenith = Zenith / SUM;
	  fZenith = (double)(Zenith*3.14/1800);
	  int16_t sonarDistance =  (-0.0163)*elapsedTime[5]+532;
	  debug[0] = sonarDistance;
	  debug[1] = tan(fZenith)*sonarDistance;
	  
	  int32_t tempx = (debug[1]*dx) >> 7;
	  int32_t tempy = (debug[2]*dy) >> 7;
	  
	  debug[2] = tempx;
	  debug[3] = tempy;
	  
	  return debug[1];
}
//Distance Calculation
void distance_calculation()
{
  SUM = 0;
  int16_t sensorData[5];
  int16_t maxSensorData = 0;
  int8_t maxSensorIdx = 0;

  int16_t minSensorData = 10000; //This needs to change if we use a different beacon (currently the max reading is around 6300-6400
  int8_t minSensorIdx = 0;

  //MDP - Still need to implement this...
  const int16_t SENSOR_UPPER_LIMIT = 6400;
  const int16_t SENSOR_LOWER_LIMIT = 500;
  
  //debug[0] = TimeoutCounter[4];
  //debug[1] = TimeoutCounter[5];
  
  for (int8_t i = 0; i <= 4; i++)
  {
	//MMS - timeoutcounter and sensortimeoutflag are used to determine if sensor values should be set to min (indicating no pulse widths being detected)
	TimeoutCounter[i]++; //increment timeout counter
	
	if(SensorTimeoutFlag[i] == LOW) //if sensor transition detected reset timeoutCounter
	{
		TimeoutCounter[i] = 0;
		SensorTimeoutFlag[i] = HIGH;
	}

    if (TimeoutCounter[i] > 60) //if beacon is off, set that sensor data to 0 to prevent old values from getting latched due to no more transitions occurring
    {
	    elapsedTime[i] = 0;
		TimeoutCounter[i] = 0;
		SensorTimeoutFlag[i] = HIGH;
    }
	
	
	sensorData[i] = elapsedTime[i];
	    
    if (sensorData[i] > maxSensorData)
    {
      if (i < 4)//Exclude the bottom sensor from this
      {
        maxSensorData = sensorData[i];
        maxSensorIdx = i;
      }
    }
    else if (sensorData[i] < minSensorData)
    {
      if (i < 4)//Exclude the bottom sensor from this
      {
        minSensorData = sensorData[i];
        minSensorIdx = i;
      }
    }
    
    SUM = SUM + sensorData[i];

  //to test horizontal IR
#ifdef TEST_IR_HORIZ
    debug[i] = sensorData[i];
#endif

  }

 //removed this for the backflip
// 	TimeoutCounter[5]++; //increment timeout counter
// 	
// 	if(SensorTimeoutFlag[5] == LOW) //if sensor transition detected reset timeoutCounter
// 	{
// 		TimeoutCounter[5] = 0;
// 		SensorTimeoutFlag[5] = HIGH;
// 	}
// 
// 	if (TimeoutCounter[5] > 120) //this implies that quad is too low and we should set the sonar value to ~20cm to make it take off
// 	{
// 		elapsedTime[5] = 31500;
// 		TimeoutCounter[5] = 0;
// 		SensorTimeoutFlag[5] = HIGH;
// 	}

  //Sonar Distance calculation
  //equation to convert sonar pulse width to height in cm
  sonarDistance =  (-0.0163)*elapsedTime[5]+532;
  
  if (sonarDistance < 25)
    sonarDistance = 25;
    
//To test sonar and downward IR
#ifdef TEST_IR_DOWN_SONAR
  debug[2] = sonarDistance*32;
  debug[3] = sensorData[4];
#endif

  //MDP - Need to think about this...
  /*
  for (int8_t i = 0; i <= 5; i++)
  {
    if (sensorData[i] > SENSOR_UPPER_LIMIT)
    {
      sensorData[i] = sensorData[i] - SENSOR_UPPER_LIMIT;
    }
    
    if (sensorData[i] < SENSOR_LOWER_LIMIT)
    {
      sensorData[i] = 0;
    }
  }
  */

  
  //============================ Estimate Relative Yaw Angle ====================================================
  //MDP
  int32_t relative_corner_angle;
  int8_t leftIdx, rightIdx;
  int8_t yaw_side = maxSensorIdx*2;

  //Depending on which is the maximum sensor, store the index of the neighbouring sensors
  //Index of IR sensors go from [0,3]
  if (maxSensorIdx == 0) //check edge case
  {
    leftIdx = 3;
    rightIdx = 1;
  }
  else if (maxSensorIdx == 3) //check edge case
  {
    leftIdx = 2;
    rightIdx = 0;
  }
  else //Otherwise it is the neighbouring indices
  {
    leftIdx = maxSensorIdx - 1;
    rightIdx = maxSensorIdx + 1;
  }

  //Determine relative angle between the two sensors with the largest values
  if (sensorData[leftIdx] > sensorData[rightIdx])
  {
    //Values were tuned experimentally, more work to be done
    relative_corner_angle = ((sensorData[maxSensorIdx] - sensorData[leftIdx])*3)/70;
  }
  else
  {
    //Values were tuned experimentally, more work to be done
    relative_corner_angle = ((sensorData[maxSensorIdx] - sensorData[rightIdx])*3)/70;
    yaw_side++;
  }

  relative_corner_angle = relative_corner_angle/2;
  
  //Limit the relative angle to 45
  if (relative_corner_angle > 45)
    relative_corner_angle = 45;

  yawIR = 0;

  //MDP - This algorithm can be simplified
  //Determine the yaw angle of the quad relative to the beacon
  //Would be interesting to use IMU gyro data along with yawIR
  if (yaw_side % 2 == 0)
  {
    yawIR = yaw_side*45 + relative_corner_angle;
  }
  else if (yaw_side == 3)
  {
    yawIR = 180 - relative_corner_angle;
  }
  else if (yaw_side == 5)
  {
    yawIR = 270 - relative_corner_angle;
  }
  else if (yaw_side == 7)
  {
    yawIR = 360 - relative_corner_angle;
  }
  else
  {
    yawIR = yaw_side*90 - relative_corner_angle;
  }
  //=====================================================================================================
  
  //Calculate Rxy for X and Y estimates
  int sensor_index = 4; //default is middle sensor
    
 //TFC - always use strongest sensor reading
 if(landFlag == HIGH)
 {
    if(sensorData[maxSensorIdx] > sensorData[4])
    {
      sensor_index = maxSensorIdx;
    }
 }
  //(Max sensor value - bottom sensor value) / (scaling factor)
  Rxy = (SENSOR_UPPER_LIMIT -  sensorData[sensor_index])/24;
  
  //MMS adding sonar distance to decouple height from bottom ir values and rxy calculation
  #ifdef SONAR_COMPENSATION  
  static uint16_t counter_compensation = 0;
  float sonar_float = (float) sonarDistance;
  float Rxy_float = (float) Rxy;  
  float ratio = sonar_float / Rxy_float;
  if(ratio < 1.3 && ratio > 0.7) //we are in the hill region of the Rxy vs sonar distance function
  {
	  Rxy = (Rxy * 6) >> 3; //: scale to 75% as initial test
	  //estimate the hill as a quadratic function centered about a height equal to Rxy
	  //divide the Rxy by this function to make it equal to a constant value ~ 1
	  //add the desired Rxy estimate
	 // Rxy_float = (Rxy_float/(-0.025*(sonar_float-Rxy_float)*(sonar_float-Rxy_float) + Rxy)) + (Rxy_float/2.0);
	 // Rxy = int32_t(Rxy_float);
	 counter_compensation++;
  }

  #endif

  if (Rxy < 0)
    Rxy = 0;
   
    //MDP - should this be changed to a lookup table? Floats are very slow on the 328
    theta = (yawIR*3.14159f)/180.0f;
    x = Rxy*cos(theta);
    y = Rxy*sin(theta);
    yawIR = yawIR - 180;////MDP - new yaw

    //Send raw data to be filtered in position controller
    //The filtering should probably be moved here
    rawDataX[0] = x;
    rawDataY[0] = y;
    rawDataRxy[0] = Rxy;
    rawDataRz[0] = z;
    
}
//MDP - best gains: 0.35,0.45  2nd Best:0.25,0.15
//MDP - Position controller variables
//MDP - Best gains w/ custom att. controller: 0.75, 0.15

//PID Tuning
//float pos_Kp = 0.75f;
//float pos_Ki = 0.0f;
//float pos_Kd = 0.15f;
//
//float rxy_Kp = 0.3f;
//float rxy_Ki = 0.0f;
//float rxy_Kd = 0.2f;//was 0.1

//Simulation PID Tuning - TFC
//float pos_Kp = 0.66183f;
//float pos_Ki = 0.06841f;
//float pos_Kd = 0.6738f;
//
//float rxy_Kp = 0.3f;
//float rxy_Ki = 0.0f;
//float rxy_Kd = 0.15f;

//PID Testing - TFC
float pos_Kp = 2.5f;
float pos_Ki = 0.3f;
float pos_Kd = 0.5f;

float rxy_Kp = 2.75f;
float rxy_Ki = 0.5f;
float rxy_Kd = 1.0f;


#ifdef HEAVY_PID
//float vert_Kp = 1.5f; //change 0.5 to 1.5 for the large quad
float vert_Kp = 0.5f; //change 0.5 to 1.5 for the large quad

#else
float vert_Kp = 0.5f; //change 0.5 to 1.5 for the large quad
#endif

float vert_Ki = 0.0;//MDP - we need to introduce an I term for vertical control
float vert_Kd = 0.75f;//was 2.25

//float vert_Kd = 0.75f;

float prev_derivativeX = 0.0f;
float prev_derivativeY = 0.0f;
float prev_derivativeZ = 0.0f;
float prev_errorX = 0.0f;
float prev_errorY = 0.0f;
float prev_errorZ = 0.0f;

float integratorX = 0.0f;
float integratorY = 0.0f;
float integratorZ = 0.0f;

float prev_errorRxy = 0.0f;
float prev_derivativeRxy = 0.0f;
float integratorRxy = 0.0f;
float max_iTerm = 0.5f;

//float alphaPID = (0.4)/(5.0+0.4);

//Filter parameter for derivative term in PID controller
float alphaPID = (0.4)/(5.0+0.4); //0.7407

//Need to pass integrator by reference for X and Y, change this to handle Z as well
float PID(float error, float& prev_error, float& prev_derivative, float& integrator)
{
  float output = 0;
  float delta_time = 0.020f;

  //MDP - if the position is within 10cm, return 0
  //if (error < 0.1)
    //return 0.0f;
  
  //Compute proportional component
  output += error * pos_Kp;

  //Compute derivative component if time has elapsed, try using the filter technique from SampleClient for D
    float derivative = (error - prev_error)/delta_time;

    derivative = (1-alphaPID)*prev_derivative + alphaPID*derivative;
    
    //update state
    prev_error = error;
    prev_derivative = derivative;

    //add in derivative component
    derivative = derivative * pos_Kd;
    output += derivative;

  //Compute integral component, deal with
  if ((pos_Ki>0) && (error > 0.05))
  {
    integrator = integrator + (error * pos_Ki) * delta_time;
    if (integrator < -1*max_iTerm)
    {
      integrator = -1*max_iTerm;
    } 
    else if (integrator >max_iTerm)
    {
      integrator = max_iTerm;
    }

    output += integrator;
  }

  return output;
}

float PIDRxy(float error, float& prev_error, float& prev_derivative, float& integrator)
{
  float output = 0;
  float delta_time = 0.020f;

  //MDP - if the position is within 10cm, return 0
  //if (error < 0.1)
    //return 0.0f;
  
  //Compute proportional component
  output += error * rxy_Kp;

  //Compute derivative component if time has elapsed, try using the filter technique from SampleClient for D
    float derivative = (error - prev_error)/delta_time;

    derivative = (1-alphaPID)*prev_derivative + alphaPID*derivative;
    
    //update state
    prev_error = error;
    prev_derivative = derivative;

    //add in derivative component
    derivative = derivative * rxy_Kd;
    output += derivative;

  //Compute integral component, deal with
  if ((pos_Ki>0) && (error > 0.05))
  {
    integrator = integrator + (error * rxy_Ki) * delta_time;
    if (integrator < -1*max_iTerm)
    {
      integrator = -1*max_iTerm;
    } 
    else if (integrator >max_iTerm)
    {
      integrator = max_iTerm;
    }

    output += integrator;
  }

  return output;
}
float PIDz(float error, float& prev_error, float& prev_derivative, float& integrator)
{
  float output = 0;
  float delta_time = 0.020f;
  
  //Compute proportional component
  output += error * vert_Kp;

  //Compute derivative component if time has elapsed, try using the filter technique from SampleClient for D
    float derivative = (error - prev_error)/delta_time;

    derivative = (1-alphaPID)*prev_derivative + alphaPID*derivative;
    
    //update state
    prev_error = error;
    prev_derivative = derivative;

    //add in derivative component
    derivative = derivative * vert_Kd;
    output += derivative;

  //Compute integral component, deal with
  if ((vert_Ki > 0) && (sonarDistance > 30))
  {
    integrator = integrator + (error * vert_Ki) * delta_time;
    if(integrator < -1*max_iTerm)
    {
      integrator = -1*max_iTerm;
    } 
    else if(integrator > max_iTerm)
    {
      integrator = max_iTerm;
    }

    output += integrator;
  }

  return output;
}

//Positions are in meters, command angles are in degrees
void Position_Control(float current_X, float current_Y, float current_Z, float& roll_command, float& pitch_command, float& thrust_command)
{
  float a_x=0, a_y=0, a_z=0; // Acceleration in each axis
  //float desX = ((float)desiredX)/100.0f;
  //float desY = ((float)desiredY)/100.0f;
//  a_x = PID((desiredX/100.0f)-current_X, prev_errorX, prev_derivativeX, integratorX);
//  a_y = PID((desiredY/100.0f)-current_Y, prev_errorY, prev_derivativeY, integratorY);
//  a_z = PIDz(desiredHeight-sonarDistance, prev_errorZ, prev_derivativeZ, integratorZ);

  a_x = PID((desiredX/100.0f)-current_X, prev_errorX, prev_derivativeX, integratorX);
  a_y = PID((desiredY/100.0f)-current_Y, prev_errorY, prev_derivativeY, integratorY);
  a_z = PIDz(desiredHeight-sonarDistance, prev_errorZ, prev_derivativeZ, integratorZ);
  //roll_command = (1/9.81)*(a_x*sin(yaw) - a_y*cos(yaw))*(180/M_PI);
  //pitch_pitch = (1/9.81)*(a_x*cos(yaw) + a_y*sin(yaw))*(180/M_PI);

  roll_command = (1/9.81)*(a_x)*(180/3.14159);
  pitch_command = (1/9.81)*(a_y)*(180/3.14159);

  if(landFlag == LOW)
  {
    //TFC thrust_command = quad_mass * (a_z + 9.81);
    //TFC thrust_command = (quad_mass * (a_z + 9.81))/(cos(roll_command)*cos(pitch_command));
    //TFC thrust_command = 4 * a_z + 1000;
    thrust_command = a_z*3; //MDP - using centimeters for Z for now. This should either be changed to meters or x,y changed to centimeters.
      //MDP - we should figure out a good limit for thrust_command
      if (thrust_command > 400)
      {
        thrust_command = 400;
      }
      else if (thrust_command < -300)
      {
        thrust_command = -300;
      }
  }
  else
  {
    //Start Landing Sequence
    //TFC - increase pitch/thrust command to force the quad to advance past car
    if((quad_behind == false) && (forwardFlag == HIGH))
    {
     pitch_command = -5;   
     thrust_command = 110; 
    }
     
    if (forwardFlag == LOW)
    {
      thrust_command = 75; // TFC - decrease thrust command to below hovering, land
      if((sonarDistance <= 30) && (Rxy < 20))//TFC - turn off rotors
      {
        thrust_command = 0;
        go_disarm(); //End Landing Sequence
      }
    }
  }
  
  float angle_limit_factor = 1;
  
  if (chasecarFlag == HIGH)
  {
    angle_limit_factor = 1.5;
  }

  if (roll_command > 5.0*angle_limit_factor)
    roll_command = 5.0*angle_limit_factor;
  else if (roll_command < -5.0*angle_limit_factor)
    roll_command = -5.0*angle_limit_factor;

  if (pitch_command > 5.0*angle_limit_factor)
    pitch_command = 5.0*angle_limit_factor;
  else if (pitch_command < -5.0*angle_limit_factor)
    pitch_command = -5.0*angle_limit_factor;
}

//Positions are in meters, command angles are in degrees
//position control which controls rxy
void Position_Control_Rxy(float currentRxy, float desiredRxy, float& roll_command, float& pitch_command, float& thrust_command)
{
	float a_y=0, a_z=0; // Acceleration in each axis
	a_y = PIDRxy(desiredRxy-currentRxy, prev_errorRxy, prev_derivativeRxy, integratorRxy); //will apply a pid on Rxy and will try to have Rxy stay at 0.5 meters
	a_z = PIDz(desiredHeight-sonarDistance, prev_errorZ, prev_derivativeZ, integratorZ);

	pitch_command = (1/9.81)*(a_y)*(180/3.14159);
  roll_command = 0;
  //TFC
	thrust_command = a_z*3; //MDP - using centimeters for Z for now. This should either be changed to meters or x,y changed to centimeters.

	//MDP - we should figure out a good limit for thrust_command
	if (thrust_command > 400)
	{
		thrust_command = 400;
	}
	else if (thrust_command < -300)
	{
		thrust_command = -300;
	}

	if (roll_command > 5)
	roll_command = 5;
	else if (roll_command < -5)
	roll_command = -5;

	if (pitch_command > 5)
	pitch_command = 5;
	else if (pitch_command < -5)
	pitch_command = -5;
}


void Keep_Distance(float currentX, float currentY, float desired_distance, float& desX, float& desY)
{
  float slope = abs(currentY/currentX);
  desX = sqrt((desired_distance*desired_distance)/(1 + (slope*slope)));
  desY = slope*desX;

  if (currentX < 0)
    desX = -1*desX;
  
  if (currentY < 0)
    desY = -1*desY; 
}

void go_arm()
{
  if (calibratingG == 0
#if defined(ONLYARMWHENFLAT)
      && f.ACC_CALIBRATED
#endif
#if defined(FAILSAFE)
      && failsafeCnt < 2
#endif
     ) {
    if (!f.ARMED && !f.BARO_MODE) { // arm now!
      f.ARMED = 1;
#if defined(HEADFREE) || defined(HEADHOLD)
      headFreeModeHold = att.heading;
      magHold = att.heading;
#else
      headFreeModeHold = att.heading;
      magHold = att.heading;
#endif
#if defined(VBAT)
      if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
#endif
#ifdef LCD_TELEMETRY // reset some values when arming
#if BARO
      BAROaltMax = alt.EstAlt;
#endif
#if GPS
      GPS_speedMax = 0;
#endif
#ifdef POWERMETER_HARD
      powerValueMaxMAH = 0;
#endif
#endif
#ifdef LOG_PERMANENT
      plog.arm++;           // #arm events
      plog.running = 1;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
      // write now.
      writePLog();
#endif
    }
  } else if (!f.ARMED) {
    blinkLED(2, 255, 1);
    alarmArray[8] = 1;
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
#ifdef LOG_PERMANENT
    plog.disarm++;        // #disarm events
    plog.armed_time = armedTime ;   // lifetime in seconds
    if (failsafeEvents) plog.failsafe++;      // #acitve failsafe @ disarm
    if (i2c_errors_count > 10) plog.i2c++;           // #i2c errs @ disarm
    plog.running = 0;       // toggle @ arm & disarm to monitor for clean shutdown vs. powercut
    // write now.
    writePLog();
#endif
  }
}

// ******** Main Loop *********
void loop ()
{
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis, i;
  int16_t error, errorAngle;
  int16_t delta;
  int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0, 0};
  static int16_t errorAngleI[2] = {0, 0};
  static int16_t delta1[3], delta2[3];
  static int32_t errorGyroI[3] = {0, 0, 0};
  static int16_t lastError[3] = {0, 0, 0};
  int16_t deltaSum;
  int16_t AngleRateTmp, RateError;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
  int16_t rc;
  int32_t prop = 0;
  static uint32_t dbg_time = 0;//MDP

  //DS ???????
  conf.activate[BOXARM] = 4;
  conf.activate[BOXHORIZON] = 4;

#if defined(SPEKTRUM)
  if (spekFrameFlags == 0x01) readSpektrum();
#endif

  currentTime = micros();
        
  if (currentTime > RCTime )    
  { // 50Hz
    //debug[2] = ((micros()/1000.0f) - dbgStartTime); //MDP - for debugging main loop time
    //dbgStartTime = micros()/1000.0f;  //MDP - for debugging purposes
    RCTime += CHECK_RCTIME;
    computeRC();
    // Failsafe routine - added by MIS
#if defined(FAILSAFE)
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && f.ARMED) {                // Stabilize, and set Throttle to specified level
      for (i = 0; i < 3; i++) rcData[i] = MIDRC;                          // after specified guard time after RC signal is lost (in 0.1sec)
      rcData[THROTTLE] = conf.failsafe_throttle;
      if (failsafeCnt > 5 * (FAILSAFE_DELAY + FAILSAFE_OFF_DELAY)) 
      {      // Turn OFF motors after specified Time (in 0.1sec)
        go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
        f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeEvents++;
    }
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && !f.ARMED) { //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
      go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
    }
    failsafeCnt++;
#endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for (i = 0; i < 4; i++)
    {
      stTmp >>= 2;
      if (rcData[i] > MINCHECK) stTmp |= 0x80;     // check for MIN
      if (rcData[i] < MAXCHECK) stTmp |= 0x40;     // check for MAX
    }
    if (stTmp == rcSticks) {
      if (rcDelayCommand < 250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) // THROTTLE at minimum
    {            
#if !defined(FIXEDWING)
      errorGyroI[ROLL] = 0;
      errorGyroI[PITCH] = 0;
      errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
#endif

      if (conf.activate[BOXARM] > 0) 
      {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) 
          go_arm(); 
        else if (f.ARMED) 
          go_disarm();
      }
    }
    else //MDP - Added this so that we would be able to disarm even when the throttle is high
    {
      if (!(rcOptions[BOXARM] && f.OK_TO_ARM))
      {
        go_disarm();
      }
    }
    
    
    if (rcDelayCommand == 20) {
      if (f.ARMED) {                  // actions during armed
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm();    // Disarm via ROLL
#endif
      } else {                        // actions during not armed
        i = 0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG = 512;
#if GPS
          GPS_reset_home_position();
#endif
#if BARO
          calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
#endif
        }
#if defined(INFLIGHT_ACC_CALIBRATION)
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
          if (AccInflightCalibrationMeasurementDone) {               // trigger saving into eeprom after landing
            AccInflightCalibrationMeasurementDone = 0;
            AccInflightCalibrationSavetoEEProm = 1;
          } else {
            AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
          }
        }
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
        if      (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) i = 1;  // ROLL left  -> Profile 1
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) i = 2;  // PITCH up   -> Profile 2
        else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) i = 3;  // ROLL right -> Profile 3
        if (i) {
          global_conf.currentSet = i - 1;
          writeGlobalSet(0);
          readEEPROM();
          blinkLED(2, 40, i);
          alarmArray[0] = i;
        }
#endif
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {            // Enter LCD config
#if defined(LCD_CONF)
          configurationLoop(); // beginning LCD configuration
#endif
          previousTime = micros();
        }
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm();      // Arm via ROLL
#endif
#ifdef LCD_TELEMETRY_AUTO
        else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {              // Auto telemetry ON/OFF
          if (telemetry_auto) {-
            telemetry_auto = 0;
            telemetry = 0;
          } else
            telemetry_auto = 1;
        }
#endif
#ifdef LCD_TELEMETRY_STEP
        else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {              // Telemetry next step
          telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
#if defined( OLED_I2C_128x64)
          if (telemetry != 0) i2c_OLED_init();
#elif defined(OLED_DIGOLE)
          if (telemetry != 0) i2c_OLED_DIGOLE_init();
#endif
          LCDclear();
        }
#endif
#if ACC
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA = 512;   // throttle=max, yaw=left, pitch=min
#endif
#if MAG
        else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
#endif
        i = 0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
          conf.angleTrim[PITCH] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
          conf.angleTrim[PITCH] -= 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
          conf.angleTrim[ROLL] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
          conf.angleTrim[ROLL] -= 2;
          i = 1;
        }
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
#if defined(LED_RING)
          blinkLedRing();
#endif
        }
      }
    }
#if defined(LED_FLASHER)
    led_flasher_autoselect_sequence();
#endif

#if defined(INFLIGHT_ACC_CALIBRATION)
    if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ) { // Copter is airborne and you are turning it off via boxarm : start measurement
      InflightcalibratingA = 50;
      AccInflightCalibrationArmed = 0;
    }
    if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
      if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone) 
      {
        InflightcalibratingA = 50;
      }
    } 
    else if (AccInflightCalibrationMeasurementDone && !f.ARMED) 
    {
      AccInflightCalibrationMeasurementDone = 0;
      AccInflightCalibrationSavetoEEProm = 1;
    }
#endif

    uint16_t auxState = 0;
    for (i = 0; i < 4; i++)
      auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
    for (i = 0; i < CHECKBOXITEMS; i++)
      rcOptions[i] = (auxState & conf.activate[i]) > 0;


    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
#if ACC
    if ( rcOptions[BOXANGLE] || (failsafeCnt > 5 * FAILSAFE_DELAY) ) {
      // bumpless transfer to Level mode
      if (!f.ANGLE_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ANGLE_MODE = 1;
      }
    } else {
      // failsafe support
      f.ANGLE_MODE = 0;
    }
    if ( rcOptions[BOXHORIZON] ) {
      f.ANGLE_MODE = 0;
      if (!f.HORIZON_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.HORIZON_MODE = 1;
      }
    } else {
      f.HORIZON_MODE = 0;
    }
#endif

#ifdef HEADFREE
    if (rcOptions[BOXHEADFREE] == 1) f.HEADFREE_MODE = 1; else f.HEADFREE_MODE = 0;
#endif

#ifdef HEADHOLD
    if (rcOptions[BOXHEADHOLD] == 1) f.HEADHOLD_MODE = 1; else f.HEADHOLD_MODE = 0;
#endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
#if !defined(GPS_LED_INDICATOR)
    if (f.ANGLE_MODE || f.HORIZON_MODE) {
      STABLEPIN_ON;
    } else {
      STABLEPIN_OFF;
    }
#endif

#if defined(PID_SWITCH)
    {
      boolean changed = false;
      if (rcOptions[BOXPID]) {
        if (!f.PID_MODE) {
          f.PID_MODE = 1;
          changed = true;
        }
      } else {
        if ( f.PID_MODE) {
          f.PID_MODE = 0;
          changed = true;
        }
      }
      if (changed) {
        conf.pid[PIDROLL].P8  = conf.pidset[f.PID_MODE][PID_ROLL].P8;
        conf.pid[PIDROLL].I8  = conf.pidset[f.PID_MODE][PID_ROLL].I8;
        conf.pid[PIDROLL].D8  = conf.pidset[f.PID_MODE][PID_ROLL].D8;
        conf.pid[PIDPITCH].P8 = conf.pidset[f.PID_MODE][PID_PITCH].P8;
        conf.pid[PIDPITCH].I8 = conf.pidset[f.PID_MODE][PID_PITCH].I8;
        conf.pid[PIDPITCH].D8 = conf.pidset[f.PID_MODE][PID_PITCH].D8;
        conf.pid[PIDLEVEL].P8 = conf.pidset[f.PID_MODE][PID_LEVEL].P8;
        // I term in oldskool, Horizon P Term in alexk
        conf.pid[PIDLEVEL].I8 = conf.pidset[f.PID_MODE][PID_LEVEL].I8;
        // not D term, but windup limit in oldskool, nothing in alexk
        conf.pid[PIDLEVEL].D8 = conf.pidset[f.PID_MODE][PID_LEVEL].D8;
      }
    }
#else
    if (PID_CONTROLLER == 1) f.PID_MODE = 0;
    else f.PID_MODE = 1;
#endif


    /*
         Ziegler–Nichols method Control Type
              Kp      Ki         Kd
          P   0.50Ku    -        -
          PI  0.45Ku  1.2 Kp/Pu  -
          PID 0.60Ku  2 Kp/Pu    KpPu/8

          Ku = input waarde 0...250
          Kp = berekende P waarde (0,6 Ku)
          Pu = de frequentie van oscillatie
    */

#if defined(DIAL_TUNING)
    int ku;
    int kp;
#if defined(POT_PR)
    ku = ((constrain(rcData[POT_PR], 1000, 2000) - 1000) << 1) >> 6 ; // * 20/640;
    kp = ku * 3 ;
    conf.pid[PIDROLL].P8 = kp;
    conf.pid[PIDROLL].I8 = (kp << 1) / FREQ_TU_PR;
    conf.pid[PIDROLL].D8 = (kp * FREQ_TU_PR) >> 3;
    conf.pid[PIDPITCH].P8 = kp;
    conf.pid[PIDPITCH].I8 = (kp << 1) / FREQ_TU_PR;
    conf.pid[PIDPITCH].D8 = (kp * FREQ_TU_PR) >> 3;
#endif
#if defined(POT_L)
    ku = ((constrain(rcData[POT_L], 1000, 2000) - 1000) << 1) >> 6 ; // * 20/640;
    kp = ku * 4 ;
    conf.pid[PIDLEVEL].P8 = kp;
    if (f.PID_MODE == 0)
      conf.pid[PIDLEVEL].I8 = (kp << 1) / FREQ_TU_PR;
    else
      conf.pid[PIDLEVEL].I8 = kp;
#endif
#endif

#if BARO
#if (!defined(SUPPRESS_BARO_ALTHOLD))
    if (rcOptions[BOXBARO]) {
      if (!f.BARO_MODE) {
        f.BARO_MODE = 1;
        AltHold = alt.EstAlt;
#if defined(ALT_HOLD_THROTTLE_MIDPOINT)
        initialThrottleHold = ALT_HOLD_THROTTLE_MIDPOINT;
#else
        initialThrottleHold = rcCommand[THROTTLE];
#endif
        errorAltitudeI = 0;
        BaroPID = 0;
      }
    } else {
      f.BARO_MODE = 0;
    }
#endif
#ifdef VARIOMETER
    if (rcOptions[BOXVARIO]) {
      if (!f.VARIO_MODE) {
        f.VARIO_MODE = 1;
      }
    } else {
      f.VARIO_MODE = 0;
    }
#endif
#endif
#if MAG
    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = att.heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    if (rcOptions[BOXHEADFREE]) {
      if (!f.HEADFREE_MODE) {
        f.HEADFREE_MODE = 1;
      }
#if defined(ADVANCED_HEADFREE)
      if ((f.GPS_FIX && GPS_numSat >= 5) && (GPS_distanceToHome > ADV_HEADFREE_RANGE) ) {
        if (GPS_directionToHome < 180)  {
          headFreeModeHold = GPS_directionToHome + 180;
        } else {
          headFreeModeHold = GPS_directionToHome - 180;
        }
      }
#endif
    } else {
      f.HEADFREE_MODE = 0;
    }
    if (rcOptions[BOXHEADADJ]) {
      headFreeModeHold = att.heading; // acquire new heading
    }
#endif

#if GPS
    static uint8_t GPSNavReset = 1;
    if (f.GPS_FIX && GPS_numSat >= 5 ) {
      if (rcOptions[BOXGPSHOME]) {  // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
        if (!f.GPS_HOME_MODE)  {
          f.GPS_HOME_MODE = 1;
          f.GPS_HOLD_MODE = 0;
          GPSNavReset = 0;
#if defined(I2C_GPS)
          GPS_I2C_command(I2C_GPS_COMMAND_START_NAV, 0);       //waypoint zero
#else // SERIAL
          GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
          nav_mode    = NAV_MODE_WP;
#endif
        }
      } else {
        f.GPS_HOME_MODE = 0;
        if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL]) < AP_MODE && abs(rcCommand[PITCH]) < AP_MODE) {
          if (!f.GPS_HOLD_MODE) {
            f.GPS_HOLD_MODE = 1;
            GPSNavReset = 0;
#if defined(I2C_GPS)
            GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD, 0);
#else
            GPS_hold[LAT] = GPS_coord[LAT];
            GPS_hold[LON] = GPS_coord[LON];
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
            nav_mode = NAV_MODE_POSHOLD;
#endif
          }
        } else {
          f.GPS_HOLD_MODE = 0;
          // both boxes are unselected here, nav is reset if not already done
          if (GPSNavReset == 0 ) {
            GPSNavReset = 1;
            GPS_reset_nav();
          }
        }
      }
    } else {
      f.GPS_HOME_MODE = 0;
      f.GPS_HOLD_MODE = 0;
#if !defined(I2C_GPS)
      nav_mode = NAV_MODE_NONE;
#endif
    }
#endif

#if defined(FIXEDWING) || defined(HELICOPTER)
    if (rcOptions[BOXPASSTHRU]) 
    {
      f.PASSTHRU_MODE = 1;
    }
    else 
    {
      f.PASSTHRU_MODE = 0;
    }
#endif

  } 
  else 
  { // not in rc loop
    static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
    if (taskOrder > 4) taskOrder -= 5;
    switch (taskOrder) {
      case 0:
        taskOrder++;
#if MAG
        if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
#endif
      case 1:
        taskOrder++;
#if BARO
        if (Baro_update() != 0 ) break;
#endif
      case 2:
        taskOrder++;
#if BARO
        if (getEstimatedAltitude() != 0) break;
#endif
      case 3:
        taskOrder++;
#if GPS
        if (GPS_Enable) GPS_NewData();
        break;
#endif
      case 4:
        taskOrder++;
#if SONAR
        Sonar_update();
#endif
#ifdef LANDING_LIGHTS_DDR
        auto_switch_landing_lights();
#endif
#ifdef VARIOMETER
        if (f.VARIO_MODE) vario_signaling();
#endif
        break;
    }
  }

  if (currentTime >= PIDTime) //Runs at 200 Hz
  {
    PIDTime += PIDCALCTIME;

    computeIMU();

    // Measure loop rate just afer reading the sensors
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

    //***********************************
    //**** Experimental FlightModes *****
    //***********************************
#if defined(ACROTRAINER_MODE)
    if (f.ANGLE_MODE) {
      if (abs(rcCommand[ROLL]) + abs(rcCommand[PITCH]) >= ACROTRAINER_MODE ) {
        f.ANGLE_MODE = 0;
        f.HORIZON_MODE = 0;
        f.MAG_MODE = 0;
        f.BARO_MODE = 0;
        f.GPS_HOME_MODE = 0;
        f.GPS_HOLD_MODE = 0;
      }
    }
#endif

#ifdef HEADHOLD
    if (f.HEADHOLD_MODE)
    {
      errorAngle = headFreeModeHold - att.heading;
      if (errorAngle >  180) errorAngle -= 360;
      if (errorAngle < -180) errorAngle += 360;
      errorAngle = constrain(errorAngle, -100, 100);
      rcCommand[YAW] += (errorAngle * conf.pid[PIDMAG].P8) >> 4;
    }
#endif

    //***********************************

#if MAG
    if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
      int16_t dif = att.heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif * conf.pid[PIDMAG].P8 >> 5;
    } else magHold = att.heading;
#endif

#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    /* Smooth alt change routine , for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease
     * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
     */
    if (f.BARO_MODE) {
      static uint8_t isAltHoldChanged = 0;
      static int16_t AltHoldCorr = 0;
      if (abs(rcCommand[THROTTLE] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
        AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
        if (abs(AltHoldCorr) > 512) {
          AltHold += AltHoldCorr / 512;
          AltHoldCorr %= 512;
        }
        isAltHoldChanged = 1;
      } 
      else if (isAltHoldChanged) 
      {
        AltHold = alt.EstAlt;
        isAltHoldChanged = 0;
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
#endif

#if defined(THROTTLE_ANGLE_CORRECTION)
    if (f.ANGLE_MODE || f.HORIZON_MODE) {
      rcCommand[THROTTLE] += throttleAngleCorrection;
    }
#endif

#if GPS
    if ( (f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME ) 
    {
      float sin_yaw_y = sin(att.heading * 0.0174532925f);
      float cos_yaw_x = cos(att.heading * 0.0174532925f);
#if defined(NAV_SLEW_RATE)
      nav_rated[LON]   += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
      nav_rated[LAT]   += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
      GPS_angle[ROLL]   = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
      GPS_angle[PITCH]  = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
#else
      GPS_angle[ROLL]   = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
      GPS_angle[PITCH]  = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
#endif
    } 
    else 
    {
      GPS_angle[ROLL]  = 0;
      GPS_angle[PITCH] = 0;
    }
#endif
      //MDP - Moved all of this outside so that the yawIR can be used for attitude yaw control...
      //Calculate X,Y,Z relative to beacon
      distance_calculation();
	  
	  #ifdef SEGEV_xy
	  distance_calculation2();
	  #endif
	  
	  #ifdef SEGEV_HEADING
	  GetPhiAngleFromIR();
	  #endif
	  
	  #ifdef SEGEV_DISTANCE
	  GetDistanceFromIR();
	  #endif
	  
	  #ifdef SEGEV_ZENITH
	  GetZenithFromIR();
	  #endif
	  
          //MDP - Need to tune these values further, it almost seems like the old yaw readings were better than the new filtered ones...investigate
      filteredYaw[1] = (1-0.1)*filteredYaw[0] + 0.1*yawIR; //Maybe alpha should be larger, 0.
      filteredYaw[0] =  filteredYaw[1];
	  
	  #ifdef SEGEV_HEADING
	  debug[0] = filteredYaw[1];
	  #endif

      //Filtering parameter for complementary filter
      float alpha = (0.2)/(1.0+0.2);


      filteredDataX[1] = (1-alpha)*filteredDataX[0] + alpha*rawDataX[0];
      filteredDataY[1] = (1-alpha)*filteredDataY[0] + alpha*rawDataY[0];
      filteredDataRxy[1] = (1-alpha)*filteredDataRxy[0] + alpha*rawDataRxy[0];
      filteredDataRz[1] = (1-alpha)*filteredDataRz[0] + alpha*rawDataRz[0];



      filteredDataX[0] = filteredDataX[1];
      filteredDataY[0] = filteredDataY[1];
      filteredDataRxy[0] = filteredDataRxy[1];
      filteredDataRz[0] = filteredDataRz[1];
	  
	  #ifdef SEGEV_xy
	  debug[2] = filteredDataX[1];
	  debug[3] = filteredDataY[1];
	  #endif
	  
	  #ifdef SEGEV_DISTANCE
	  debug[1] = filteredDataRxy[0]; //post filtered RXY
	  #endif
	  

      

	  static int16_t backflipCounter = -300; 
    //MDP - Run position controller, probably only want to run this at maybe 1/4 of the rate
    if (pos_controller_counter == 3) //Runs at 1/4 of attitude speed
    {
   

      float current_x = (filteredDataX[0]/100.0f);
      float current_y = (filteredDataY[0]/100.0f);
      float current_z = (filteredDataRz[0]/100.0f);

      //MDP - User sets the desired height according to throttle maps [1000, 2000] --> [200, 400]. Verify the incoming rcData[THROTTLE] values.
      desiredHeight = (rcDataCmds[THROTTLE]/5) - 150;//Used for the PIDz controller

    //the Keep_Distance function takes the currentx and currenty location values and will set the desiredX and desiredY values to the closest point on the
    //circle with radius equal to the third function parameter
    static uint8_t keepdistCounter = 5;
	
//     if (keepdistFlag == HIGH)
//     {
//       if (false)//(keepdistCounter == 5)
//       {
//         Keep_Distance(current_x, current_y, 100, desiredX, desiredY);
//         keepdistCounter = 0;
//         //keepdistCounter++;
//       }
//       Keep_Distance(current_x, current_y, 75, desiredX, desiredY);
//       //desiredX = 0;
//       //desiredY = -75;
//       //keepdistCounter++;
//     }
//     else //if we don't have keep distance enabled then desired position is commanded by the RC commands from the APP
//     {
//       desiredX = (rcDataCmds[0]/5) - 300;// [1000, 2000] --> [200,400] --> [-100,100]
//       desiredY = (rcDataCmds[1]/5) - 300;
//     }

      #ifdef MDP_TEST_TEMP
        debug[0] = filteredDataX[0];
        debug[1] = filteredDataX[1];
        debug[2] = sonarDistance;
        //debug[3] = rcDataCmds[3];
        //debug[2] = desiredX*100;
        //debug[3] = desiredY*100;
      #endif

      //Put into meters...
      //desiredX = desiredX/100.0f;
      //desiredY = desiredY/100.0f;

       
        
	    if (keepdistFlag == HIGH)
		  {
			  Position_Control_Rxy(filteredDataRxy[1]/100.0f, 0.4, roll_cmd, pitch_cmd, thrust_cmd); //will apply PID on only Rxy and will only change pitch. To be used with yaw PID.	
		  }
		  else
		  {
			  desiredX = (rcDataCmds[0]/5) - 300;// [1000, 2000] --> [200,400] --> [-100,100]
			  desiredY = (rcDataCmds[1]/5) - 300;
       if (chasecarFlag == HIGH)
       {
        pos_Kp = 1.25;
        pos_Kd = 0.05;
        vert_Kp = 1.15;
       }
       else if (flipFlag == HIGH)
       {
          if (backflipCounter < 0)
          {
            desiredHeight = 50;
          }
       }
       else
       {
        vert_Kp = 0.5;
        pos_Kp = 0.75;
        pos_Kd = 0.15;
       }
			  Position_Control(current_x, current_y, current_z, roll_cmd, pitch_cmd, thrust_cmd); //IMPORTANT: this function now uses desiredX and desiredY global variables as target coordinates
		  }
		  
	  //the following code will activate if AUX2 is set to 1500 from any other value
	  static uint16_t wobbleCounter = 0;	
    static uint16_t wobbleCounter2 = 0; //TFC  
	  static uint16_t polarity = 1;  	  
	  if(hitFlag == HIGH)
	  {
      landFlag = HIGH;    //TFC
      forwardFlag = HIGH; //TFC
    // - should we set specific gains here or in the controller?
    //ASH- Removed the functionality of the arrow button
    /*
		  if(wobbleCounter % 6 == 0)	//every three loops it will change the sign of the pitch and roll degrees
		  {
	  		  polarity = polarity * -1;
		  }
		  pitch_cmd = polarity*12; //will alternate between plus and minus 3 degrees every 3 loops
		  roll_cmd = polarity*12;
		  vert_Kp = 1.0;
       */
		  wobbleCounter++;
     
		  if (wobbleCounter == 40 || quad_behind == false) //reached end of timer for forward
		  {
			  wobbleCounter = 0;
        forwardFlag = LOW;
        wobbleCounter2++;
		  }	
      if(wobbleCounter2 == 40) // reached end of timer for landing
      {
        wobbleCounter2 = 0;
        landFlag = LOW; // Back to the normal sequence 
        hitFlag = LOW;
        //go_disarm();
        //vert_Kp = 0.5;
      }
	  }

    static uint16_t swingCounter = 0;  
    if(swingFlag == HIGH)
    {
      if(swingCounter < 5)  //every three loops it will change the sign of the pitch and roll degrees
      {
          roll_cmd = 30;
      }
      else
      {
        roll_cmd = -30;
      }
      pitch_cmd = 0; //will alternate between plus and minus 3 degrees every 3 loops

      //vert_Kp = 1.0;
      swingCounter++;
      if (swingCounter == 10) //reached end of timer
      {
        swingCounter = 0;
        swingFlag = LOW;
        //vert_Kp = 0.5;
      } 
    }

    //the following code will activate a yaw dance move
      static uint16_t danceYAWcounter = 0;
      if(danceYAWflag == HIGH)
      {  
          vert_Kp = 1.25;
          if(danceYAWcounter % 20 == 6)
          {
               polarity = -1;
          }
          if(danceYAWcounter % 20 == 0)
          {
              polarity = 1;              
          }
          pitch_cmd = 0;
          roll_cmd = 0;
          desiredDanceYaw = polarity * 500 + 1500;
         
          danceYAWcounter++;
          if(danceYAWcounter == 24)
          {
              danceYAWcounter = 0;
              danceYAWflag = LOW;
              vert_Kp = 0.5;
          }
      }
      //MDP - Override for hovering - DEBUG ONLY, REMOVE BEFORE FLIGHT!
      //roll_cmd = 0;
      //pitch_cmd = 0;
      
      //MDP - With the yaw estimate technique the pitch angle is inverted!
      pitch_cmd = -1*pitch_cmd;

      //debug[0] = filteredDataRxy[0];
      //debug[1] = filteredDataX[0];
      //debug[2] = filteredDataY[0];
      //debug[3] = filteredYaw[0];
      
      pos_controller_counter = 0;
    }
    else
    {
      pos_controller_counter++;
    }


    //MDP - This is the active PID controller for attitude
    //**** PITCH & ROLL & YAW PID ****
	//during the first part of the backflip set the attitude angles to zero to maintain flat quad and increase thrust to make quad gain altitude
	//after the backflip is complete set roll and pitch to zero and 
	if (flipFlag == HIGH && backflipCounter > 0) 
	{
		thrust_cmd = 250;
		if(backflipCounter < 300) //ignore position controller and just set the quad to flat angles
		{
		roll_cmd = 0;
		pitch_cmd = 0;			
		}
	}
    if (f.PID_MODE == 0) 
    { // evolved oldschool  **********************

      if ( f.HORIZON_MODE ) prop = min(max(abs(pitch_cmd*10.0f), abs(roll_cmd*10.0f)), 512);

      //Roll and Pitch (0,1)
      for (axis = 0; axis < 2; axis++)
      {
        rc = rcCommand[axis] << 1;
        
        error = rc - imu.gyroData[axis];
        errorGyroI[axis]  = constrain(errorGyroI[axis] + error, -16000, +16000);   // WindUp   16 bits is ok here
        if (abs(imu.gyroData[axis]) > 640) errorGyroI[axis] = 0;

        ITerm = (errorGyroI[axis] >> 7) * conf.pid[axis].I8 >> 6;                  // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

        PTerm = (int32_t)rc * conf.pid[axis].P8 >> 6;

        if (f.ANGLE_MODE || f.HORIZON_MODE) //MDP - This is active
        { // axis relying on ACC
          // 50 degrees max inclination

          //MDP - Set angles for error calculation from the position controller
          if (!userControlFlag)
          {
            if (axis == 0)
              rc = roll_cmd*10;
            else
              rc = pitch_cmd*10;
          }
          
          errorAngle         = constrain(rc /*+ GPS_angle[axis]*/, -500, +500) - att.angle[axis] /*+ conf.angleTrim[axis]/* MDP COMMENTED THIS OUT FOR DEBUG*/; //16 bits is ok here
          errorAngleI[axis]  = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);                                            // WindUp     //16 bits is ok here
          
          PTermACC           = ((int32_t)errorAngle * conf.pid[PIDLEVEL].P8) >> 7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result

          int16_t limit      = conf.pid[PIDLEVEL].D8 * 5;
          PTermACC           = constrain(PTermACC, -limit, +limit);

          ITermACC           = ((int32_t)errorAngleI[axis] * conf.pid[PIDLEVEL].I8) >> 12; // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

          ITerm              = ITermACC + ((ITerm - ITermACC) * prop >> 9);
          PTerm              = PTermACC + ((PTerm - PTermACC) * prop >> 9);
        }

        PTerm -= ((int32_t)imu.gyroData[axis] * dynP8[axis]) >> 6; // 32 bits is needed for calculation

        delta          = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastGyro[axis] = imu.gyroData[axis];
        DTerm          = delta1[axis] + delta2[axis] + delta;
        delta2[axis]   = delta1[axis];
        delta1[axis]   = delta;
        
        DTerm = ((int32_t)DTerm * dynD8[axis]) >> 5;    // 32 bits is needed for calculation
        
        //MDP - Test custom gains for roll and pitch
        if (!userControlFlag)
        {
          if (axis < 2)
          {
            //Best gains: 1.0,  40
            PTerm = errorAngle*1.0;
            DTerm = -1*(errorAngle - prevAttError[axis])*200.0*0.4; //*200 for 5ms, then * gain of 0.5 = *20 //mms was 30
            prevAttError[axis] = errorAngle;
          }
        }

        
        axisPID[axis] =  PTerm + ITerm - DTerm;
      }
      
      //YAW
#define GYRO_P_MAX 300
#define GYRO_I_MAX 250
//static int16_t yawIRintegral = 0;
static int16_t prev_yawIR = 0;

      //MDP - 
      static float yawIntegrated = yawIR;//filteredYaw[1];
      /*float fused_yawIR = 0;
      #define yaw_filter_weight 0.85
      yawIntegrated += (-1*imu.gyroData[YAW]/4)*(((float)(cycleTime)/1000.0f)/1000.0f);//cycleTim is in micro seconds, divide by a million to get it in seconds//Have to deal with wrap around, modulus to bring it back to appropriate range
      
      if (yawIntegrated > 179)
        yawIntegrated = yawIntegrated - 360;
      else if (yawIntegrated < -179)
        yawIntegrated = yawIntegrated + 360;

      if (yawIntegrated > 90 && filteredYaw[0] < -90)
      {
        float temp_Int = yawIntegrated - 180;
        float temp_IR = filteredYaw[0] + 180;

        fused_yawIR = (1 - yaw_filter_weight)*temp_IR + (yaw_filter_weight)*temp_Int;
        if (fused_yawIR < 0)
          fused_yawIR = 180 + fused_yawIR;
        else
          fused_yawIR = -180 + fused_yawIR;
      }
      else if (yawIntegrated < -90 && filteredYaw[0] > 90)
      {
        float temp_Int = yawIntegrated + 180;
        float temp_IR = filteredYaw[0] - 180;

        fused_yawIR = (1 - yaw_filter_weight)*temp_IR + (yaw_filter_weight)*temp_Int;
        if (fused_yawIR < 0)
          fused_yawIR = 180 + fused_yawIR;
        else
          fused_yawIR = -180 + fused_yawIR;
      }
      else
      {
        fused_yawIR = (1 - yaw_filter_weight)*filteredYaw[0] + (yaw_filter_weight)*yawIntegrated;
      }*/
 
      //fused_yawIR = (1 - 0.9)*filteredYaw[0] + (0.9)*(imu.gyroData[YAW]);

      if (keepdistFlag == HIGH)
      {
       axisPID[YAW] = -1*constrain((desiredYaw - filteredYaw[0])*3.0 + (((float)imu.gyroData[YAW])/4.0)*1.0,-200,200); //*1.5; //1.5 was best response if we were using the att.heading value from multiwii
      }
      else
      {
      rc = (int32_t)rcCommand[YAW] * (2 * conf.yawRate + 30)  >> 5;

      error = rc - imu.gyroData[YAW];//rc - imu.gyroData[YAW];
      
      errorGyroI[YAW]  += (int32_t)error * conf.pid[PIDYAW].I8;
      errorGyroI[YAW]  = constrain(errorGyroI[YAW], 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
      
      if (abs(rc) > 50) errorGyroI[YAW] = 0;

      PTerm = (int32_t)error * conf.pid[PIDYAW].P8 >> 6;
      int16_t limit = GYRO_P_MAX - conf.pid[PIDYAW].D8;
      PTerm = constrain(PTerm, -limit, +limit);
      ITerm = constrain((int16_t)(errorGyroI[YAW] >> 13), -GYRO_I_MAX, +GYRO_I_MAX);
      axisPID[YAW] =  PTerm + ITerm;
      }
	  }
    else /*if (0 == 1)*/// MDP - disable this, I don't think it is entering this loop but it is tough to tell with all the multiwii flags
    { // AlexK *******************
      // Angle is Level_P
      // Horizon is Level_I
      // Level_D is unused

#define GYRO_I_MAX 256
#define ACC_I_MAX 256

      prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]
      
      //----------PID controller----------
      for (axis = 0; axis < 3; axis++) 
      {
        //-----Get the desired angle rate depending on flight mode
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2 ) { // MODE relying on ACC
          // calculate error and limit the angle to 50 degrees max inclination
          errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
        }
        if (axis == 2) {//YAW is always gyro-controlled (MAG correction is applied to rcCommand)
          AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[2]) >> 5);
        } else {
          if (!f.ANGLE_MODE) {//control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
            AngleRateTmp = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
            if (f.HORIZON_MODE) {
              //mix up angle error to desired AngleRateTmp to add a little auto-level feel
              AngleRateTmp += ((int32_t) errorAngle * conf.pid[PIDLEVEL].I8) >> 8;
            }
          } else {//it's the ANGLE mode - control is angle based, so control loop is needed
            AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8) >> 4;
          }
        }

        //--------low-level gyro-based PID. ----------
        //Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        //-----calculate scaled error.AngleRates
        //multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp  - imu.gyroData[axis];

        //-----calculate P component
        PTerm = ((int32_t) RateError * conf.pid[axis].P8) >> 7;

        //-----calculate I component
        //there should be no division before accumulating the error to integrator, because the precision would be reduced.
        //Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        //Time correction (to avoid different I scaling for different builds based on average cycle time)
        //is normalized to cycle time = 2048.
        errorGyroI[axis]  += (((int32_t) RateError * cycleTime) >> 11) * conf.pid[axis].I8;
        //limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        //I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta          = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        //Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = ((int32_t) delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;
        //add moving average here to reduce noise
        deltaSum       = delta1[axis] + delta2[axis] + delta;
        delta2[axis]   = delta1[axis];
        delta1[axis]   = delta;

        DTerm = (deltaSum * conf.pid[axis].D8) >> 8;

        //-----calculate total PID output
        axisPID[axis] =  PTerm + ITerm + DTerm;
      }
    } // f.PID_MODE **********************

    //Only control height when the quad is armed and slightly above the minimum value.
    //if (rcData[THROTTLE] > 1100)
      rcCommand[THROTTLE] = 1650 + thrust_cmd; //MDP - A guess at the hover speed throttle (1500)
    //else
    //  rcCommand[THROTTLE] = MINCHECK;

    mixTable();
    // do not update servos during unarmed calibration of sensors which are sensitive to vibration
    if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
	
	
	//voltage compensation -- adjusts PWM values based on current battery voltage
	float alpha_bat = 16.0 * (-0.02381 *  (float)analog.vbat + 1.871); //Amirali - Voltage comp.		
	if(alpha_bat > 16.0)
	{
		alpha_bat = 16.0;
	}
	uint16_t alpha_int = (int16_t) alpha_bat;
		
	//the following backflip code will activate if AUX2 is set to 2000 from any other value
	if(flipFlag == HIGH)
	{
		if(backflipCounter == 160) //after 80 loops of increasing altitude, start the backflip
		{
			motor[1] = 2000;
			motor[3] = 2000;
			motor[2] = 1000;
			motor[0] = 1000;
			writeMotors(alpha_int); //passing the PWM scaling value		
			delay(250); //gives us ability to fine tune the time for backflip //168 and 187 180 //note make this to 250
			motor[1] = 1000;
			motor[3] = 1000;
			motor[2] = 2000;
			motor[0] = 2000;
			writeMotors(alpha_int); //give it counter torque to stop rotation	
			delay(200);
     //MDP - For lower ceilings increase vert_Kp to prevent quad from hitting the ground
     vert_Kp = 2.0;
		}
   
		if(backflipCounter == 350) //end of backflip sequence
		{
      vert_Kp = 0.5;
			backflipCounter = 0;
			flipFlag = LOW;
		}				
		backflipCounter++;
	}
    writeMotors(alpha_int); //passing the pwm scaling value
  }
}
