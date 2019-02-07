#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

void initializeSoftPWM(void);

#if defined(SERVO)
void initializeServo();
#endif

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  uint8_t PWM_PIN[8] = {9,10,5,6,11,13};   //for a quad+: rear,right,left,front
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() 
{
  
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors(uint16_t batteryComp) { // [1000;2000] => [125;250]
  #if !(NUMBER_MOTOR == 4) 
    #error "only 4 motors allowed"
  #endif
  
  #if defined(MEGA)
    #error "only arduino 328 or 32u4"
  #endif

  #if defined(PROMINI)
    #if defined(EXT_MOTOR_32KHZ)
      OCR1A = (batteryComp * ((motor[0] - 1000) >> 2)) >> 4; //  pin 9
      OCR1B = (batteryComp * ((motor[1] - 1000) >> 2)) >> 4; //  pin 10
      OCR2A = (batteryComp * ((motor[2] - 1000) >> 2)) >> 4; //  pin 11
      OCR2B = (batteryComp * ((motor[3] - 1000) >> 2)) >> 4; //  pin 3
//       OCR1A = (motor[0] - 1000) >> 2; //  pin 9
//       OCR1B = (motor[1] - 1000) >> 2; //  pin 10
//       OCR2A = (motor[2] - 1000) >> 2; //  pin 11
//       OCR2B = (motor[3] - 1000) >> 2; //  pin 3
    #elif defined(EXT_MOTOR_4KHZ)
      OCR1A = (motor[0] - 1000) << 1; //  pin 9
      OCR1B = (motor[1] - 1000) << 1; //  pin 10
      OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      OCR2B = (motor[3] - 1000) >> 2; //  pin 3
    #elif defined(EXT_MOTOR_1KHZ)
      OCR1A = (motor[0] - 1000) << 3; //  pin 9
      OCR1B = (motor[1] - 1000) << 3; //  pin 10
      OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      OCR2B = (motor[3] - 1000) >> 2; //  pin 3
    #else
      #error only 32khz or 4khz or 1khz on 328 device
    #endif    
  #endif
  
  #if defined(PROMICRO)
    uint16_t Temp2;
    Temp2 = motor[3] - 1000;
    #if defined(EXT_MOTOR_64KHZ)
      OCR1A = (motor[0] - 1000) >> 2; // max = 255
      OCR1B = (motor[1] - 1000) >> 2;
      OCR3A = (motor[2] - 1000) >> 2;
      Temp2 = Temp2 >> 2;
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #elif defined(EXT_MOTOR_32KHZ)
      OCR1A = (motor[0] - 1000) >> 1; // max = 511
      OCR1B = (motor[1] - 1000) >> 1;
      OCR3A = (motor[2] - 1000) >> 1;
      Temp2 = Temp2 >> 1;
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #elif defined(EXT_MOTOR_16KHZ)
      OCR1A = motor[0] - 1000;        //  pin 9
      OCR1B = motor[1] - 1000;        //  pin 10
      OCR3A = motor[2] - 1000;        //  pin 5
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #elif defined(EXT_MOTOR_8KHZ)
      OCR1A = (motor[0]-1000) << 1;   //  pin 9
      OCR1B = (motor[1]-1000) << 1;   //  pin 10
      OCR3A = (motor[2]-1000) << 1;   //  pin 5
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #else
      #error only 32khz or 16khz or 8khz on 32u4 device
    #endif
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors(16);
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  #if defined(PROMINI)
  
    TCCR1A = (1<<WGM11); // phase correct mode & no prescaler
    TCCR1B = (1<<WGM13) | (1<<CS10);
    
    TIMSK2 = (1 << TOIE2); //Amirali
    #if defined(EXT_MOTOR_32KHZ)
      ICR1   = 0x00FF; // TOP to 255;
      TCCR2B =  (1<<CS20);
    #elif defined(EXT_MOTOR_4KHZ)
      ICR1   = 0x07F8; // TOP to 1023;     
      TCCR2B =  (1<<CS21);
    #elif defined(EXT_MOTOR_1KHZ)
      ICR1   = 0x1FE0; // TOP to 8184;     
      TCCR2B =  (1<<CS20) | (1<<CS21);
    #else
      #error only 32khz or 4khz or 1khz on 328 device
    #endif    
    TCCR1A |= _BV(COM1A1); // connect pin  9 to timer 1 channel A
    TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    TCCR2A |= _BV(COM2B1); // connect pin  3 to timer 2 channel B
  #endif

  #if defined(PROMICRO)
    TCCR1A = (1<<WGM11);
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
  
    TCCR3A = (1<<WGM31);
    TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30);
  
    #if defined(EXT_MOTOR_64KHZ)
      ICR1   = 0x00FF; // TOP to 255;
      ICR3   = 0x00FF; // TOP to 255;
      TC4H = 0x00;
      OCR4C = 0xFF; // phase and frequency correct mode & top to 255
      TCCR4B = (1<<CS40);             // prescaler to 1
    #elif defined(EXT_MOTOR_32KHZ)
      ICR1   = 0x01FF; // TOP to 511;
      ICR3   = 0x01FF; // TOP to 511;
      TC4H = 0x01;
      OCR4C = 0xFF; // phase and frequency correct mode & top to 511
      TCCR4B = (1<<CS40);             // prescaler to 1
    #elif defined(EXT_MOTOR_16KHZ)
      ICR1   = 0x03FF; // TOP to 1023;
      ICR3   = 0x03FF; // TOP to 1023;
      TC4H = 0x03;
      OCR4C = 0xFF; // phase and frequency correct mode & top to 1023
      TCCR4B = (1<<CS40);             // prescaler to 1
    #elif defined(EXT_MOTOR_8KHZ)
      ICR1   = 0x07FF; // TOP to 2046;
      ICR3   = 0x07FF; // TOP to 2046;
      TC4H = 0x3;
      OCR4C = 0xFF; // phase and frequency correct mode
      TCCR4B = (1<<CS41);             // prescaler to 2
    #else
      #error only 64khz to 8khz on 32u4 device
    #endif
  
    TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
  
    TCCR4D = 0;
    TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  
  #if defined(SERVO)
    initializeServo();
  #endif
}

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
  return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

// int8_t servodir(uint8_t n, uint8_t b) { return ((conf.servoConf[n].rate & b) ? -1 : 1) ; }

void mixTable() 
{
  int16_t maxMotor;
  uint8_t i;
  #if defined(DYNBALANCE)
    return;
  #endif
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  #define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

  /****************                   main Mix Table                ******************/
  #if defined( MY_PRIVATE_MIXING )
    #include MY_PRIVATE_MIXING
  #elif defined( TRI )
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = (SERVODIR(5, 1) * axisPID[YAW]) + get_middle(5); //REAR
  #elif defined( QUADP )
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #elif defined( Y4 )
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #elif defined( VTAIL4 )
    motor[0] = PIDMIX(+0,+1, +1); //REAR_R
    motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
    motor[2] = PIDMIX(+0,+1, -1); //REAR_L
    motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
  #elif defined( GIMBAL )
    for(i=0;i<2;i++) {
      servo[i]  = ((int32_t)conf.servoConf[i].rate * att.angle[1-i]) /50L;
      servo[i] += get_middle(i);
    }
  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif // MY_PRIVATE_MIXING



  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) 
    {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      
      if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)//MDP - this will mean that the height 
      #ifndef MOTOR_STOP
        motor[i] = conf.minthrottle;
      #else
        motor[i] = MINCOMMAND;
      #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }

  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
  {
    static uint32_t lastRead = currentTime;
    uint16_t amp;
    uint32_t ampsum, ampus; // pseudo ampere * microseconds
    /* true cubic function;
     * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
     * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
     * */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (analog.vbat > NO_VBAT) { // by all means - must avoid division by zero
      ampsum = 0;
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / analog.vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
        ampus = ( (currentTime-lastRead) * (uint32_t)amp * (uint32_t)conf.pint2ma ) / PLEVELDIVSOFT;
        #if (LOG_VALUES >= 3)
          pMeter[i]+= ampus; // sum up over time the mapped ESC input
        #endif
        #if defined(POWERMETER_SOFT)
          ampsum += ampus; // total sum over all motors
        #endif
      }
      #if defined(POWERMETER_SOFT)
        pMeter[PMOTOR_SUM]+= ampsum / NUMBER_MOTOR; // total sum over all motors
      #endif
    }
    lastRead = currentTime;
  }
  #endif
}

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delay(60); //wait 60 ms
  }
}

