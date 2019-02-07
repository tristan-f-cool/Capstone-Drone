#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiWii.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

#if defined(SPEKTRUM)
  #include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner. 
#endif

//RAW RC values will be store here
#if defined(SBUS)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#elif defined(SPEKTRUM) || defined(SERIAL_SUM_PPM)
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#else
  volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
#endif

#if defined(SERIAL_SUM_PPM) //Channel order for PPM SUM RX Configs
  static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};
#elif defined(SBUS) //Channel order for SBUS RX Configs
  // for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiWii, but it should be easy to integrate them.
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11,12,13,14,15,16,17};
  static uint16_t sbusIndex=0;
#elif defined(SPEKTRUM)
  static uint8_t rcChannel[RC_CHANS] = {PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11};
#else // Standard Channel order
  static uint8_t rcChannel[RC_CHANS]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
  static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit
#endif

void rxInt(void);

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  #if defined(STANDARD_RX)
    #if defined(MEGA)
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
    #endif
    // PCINT activation
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
    
    /*************    atmega328P's Specific Aux2 Pin Setup    *********************/
    #if defined(PROMINI)
     #if defined(RCAUXPIN)
        PCICR  |= (1 << 0) ; // PCINT activated also for PINS [D8-D13] on port B
        #if defined(RCAUXPIN8)
          PCMSK0 = (1 << 0);
        #endif
        #if defined(RCAUXPIN12)
          PCMSK0 = (1 << 4);
        #endif
      #endif
    #endif
    
    /***************   atmega32u4's Specific RX Pin Setup   **********************/
    #if defined(PROMICRO)
      //Trottle on pin 7
      DDRE &= ~(1 << 6); // pin 7 to input
      PORTE |= (1 << 6); // enable pullups
      EICRB |= (1 << ISC60);
      EIMSK |= (1 << INT6); // enable interuppt
      // Aux2 pin on PBO (D17/RXLED)
      #if defined(RCAUX2PIND17)
        DDRB &= ~(1 << 0); // set D17 to input 
      #endif
      // Aux2 pin on PD2 (RX0)
      #if defined(RCAUX2PINRXO)
        DDRD &= ~(1 << 2); // RX to input
        PORTD |= (1 << 2); // enable pullups
        EICRA |= (1 << ISC20);
        EIMSK |= (1 << INT2); // enable interuppt
      #endif
    #endif
    
  /*************************   Special RX Setup   ********************************/
  #endif
  // Init PPM SUM RX
  #if defined(SERIAL_SUM_PPM)
    PPM_PIN_INTERRUPT; 
  #endif
  // Init Sektrum Satellite RX
  #if defined (SPEKTRUM)
    SerialOpen(SPEK_SERIAL_PORT,115200);
  #endif
  // Init SBUS RX
  #if defined(SBUS)
    SerialOpen(1,100000);
  #endif
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/
#if defined(STANDARD_RX)

#if defined(FAILSAFE) && !defined(PROMICRO)
   // predefined PC pin block (thanks to lianj)  - Version with failsafe
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
        dTime = cTime-edgeTime[pin_pos];                             \
        if (900<dTime && dTime<2200) {                               \
          rcValue[rc_value_pos] = dTime;                             \
          if((rc_value_pos==THROTTLEPIN || rc_value_pos==YAWPIN ||   \
              rc_value_pos==PITCHPIN || rc_value_pos==ROLLPIN)       \
              && dTime>FAILSAFE_DETECT_TRESHOLD)                     \
                GoodPulses |= (1<<rc_value_pos);                     \
        }                                                            \
      } else edgeTime[pin_pos] = cTime;                              \
    }
#else
   // predefined PC pin block (thanks to lianj)  - Version without failsafe
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
        dTime = cTime-edgeTime[pin_pos];                             \
        if (900<dTime && dTime<2200) {                               \
          rcValue[rc_value_pos] = dTime;                             \
        }                                                            \
      } else edgeTime[pin_pos] = cTime;                              \
    }
#endif

  // port change Interrupt
  ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  #if defined(FAILSAFE) && !defined(PROMICRO)
    static uint8_t GoodPulses;
  #endif
  
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
   
    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,2);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,4);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,5);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,6);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,7);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,0);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,1);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,3);
    #endif
    
    #if defined(FAILSAFE) && !defined(PROMICRO)
      if (GoodPulses==(1<<THROTTLEPIN)+(1<<YAWPIN)+(1<<ROLLPIN)+(1<<PITCHPIN)) {  // If all main four chanells have good pulses, clear FailSafe counter
        GoodPulses = 0;
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; 
      }
    #endif
  }
  /*********************      atmega328P's Aux2 Pins      *************************/
  #if defined(PROMINI)
    #if defined(RCAUXPIN)
    /* this ISR is a simplification of the previous one for PROMINI on port D
       it's simplier because we know the interruption deals only with one PIN:
       bit 0 of PORT B, ie Arduino PIN 8
       or bit 4 of PORTB, ie Arduino PIN 12
     => no need to check which PIN has changed */
    ISR(PCINT0_vect) {
      uint8_t pin;
      uint16_t cTime,dTime;
      static uint16_t edgeTime;
    
      pin = PINB;
      cTime = micros();
      sei();
      #if defined(RCAUXPIN8)
       if (!(pin & 1<<0)) {     //indicates if the bit 0 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
      #endif
      #if defined(RCAUXPIN12)
       if (!(pin & 1<<4)) {     //indicates if the bit 4 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
      #endif
        dTime = cTime-edgeTime; if (900<dTime && dTime<2200) rcValue[0] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
      } else edgeTime = cTime;    // if the bit 2 is at a high state (ascending PPM pulse), we memorize the time
    }
    #endif
  #endif
  
  /****************      atmega32u4's Throttle & Aux2 Pin      *******************/
  #if defined(PROMICRO)
    // throttle
    ISR(INT6_vect){ 
      static uint16_t now,diff;
      static uint16_t last = 0;
      now = micros();  
      if(!(PINE & (1<<6))){
        diff = now - last;
        if(900<diff && diff<2200){
          rcValue[3] = diff;
          #if defined(FAILSAFE)
           if(diff>FAILSAFE_DETECT_TRESHOLD) {        // if Throttle value is higher than FAILSAFE_DETECT_TRESHOLD
             if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
           }
          #endif 
        }
      }else last = now; 
    }
    // Aux 2
    #if defined(RCAUX2PINRXO)
      ISR(INT2_vect){
        static uint16_t now,diff;
        static uint16_t last = 0; 
        now = micros();  
        if(!(PIND & (1<<2))){
          diff = now - last;
          if(900<diff && diff<2200) rcValue[7] = diff;
        }else last = now;
      }
    #endif  
  #endif
#endif


/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/
// attachInterrupt fix for promicro
#if defined(PROMICRO) && defined(SERIAL_SUM_PPM)
  ISR(INT6_vect){rxInt();}
#endif

// PPM_SUM at THROTTLE PIN on MEGA boards
#if defined(PPM_ON_THROTTLE) && defined(MEGA) && defined(SERIAL_SUM_PPM)
  ISR(PCINT2_vect) { if(PINK & (1<<0)) rxInt(); }
#endif

// Read PPM SUM RX Data
#if defined(SERIAL_SUM_PPM)
  void rxInt(void) {
    uint16_t now,diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;
  #if defined(FAILSAFE)
    static uint8_t GoodPulses;
  #endif
  
    now = micros();
    sei();
    diff = now - last;
    last = now;
    if(diff>3000) chan = 0;
    else {
      if(900<diff && diff<2200 && chan<RC_CHANS ) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan] = diff;
        #if defined(FAILSAFE)
          if(chan<4 && diff>FAILSAFE_DETECT_TRESHOLD) GoodPulses |= (1<<chan); // if signal is valid - mark channel as OK
          if(GoodPulses==0x0F) {                                               // If first four chanells have good pulses, clear FailSafe counter
            GoodPulses = 0;
            if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
          }
        #endif
      }
    chan++;
  }
}
#endif

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#if defined(SBUS)
void  readSBus(){
  #define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
  static uint16_t sbus[25]={0};
  while(SerialAvailable(SBUS_SERIAL_PORT)){
    int val = SerialRead(SBUS_SERIAL_PORT);
    if(sbusIndex==0 && val != SBUS_SYNCBYTE)
      continue;
    sbus[sbusIndex++] = val;
    if(sbusIndex==25){
      sbusIndex=0;
      rcValue[0]  = ((sbus[1]|sbus[2]<< 8) & 0x07FF)/2+SBUS_MID_OFFSET;
      rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF)/2+SBUS_MID_OFFSET;
      rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF)/2+SBUS_MID_OFFSET; // & the other 8 + 2 channels if you need them
      //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
      rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+SBUS_MID_OFFSET; 
      rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+SBUS_MID_OFFSET; 
      // now the two Digital-Channels
      if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
      if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;

      // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
      #if defined(FAILSAFE)
      if (!((sbus[23] >> 3) & 0x0001))
        {if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
      #endif
    }
  }        
}
#endif


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
#if defined(SPEKTRUM)
void readSpektrum(void) {
  while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE) { // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
    for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++) {SerialRead(SPEK_SERIAL_PORT);}  //Toss one full frame of bytes.
  }  
  if (spekFrameFlags == 0x01) {   //The interrupt handler saw at least one valid frame start since we were last here. 
    if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE) {  //A complete frame? If not, we'll catch it next time we are called. 
      SerialRead(SPEK_SERIAL_PORT); SerialRead(SPEK_SERIAL_PORT);        //Eat the header bytes 
      for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2) {
        uint8_t bh = SerialRead(SPEK_SERIAL_PORT);
        uint8_t bl = SerialRead(SPEK_SERIAL_PORT);
        uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
        if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
      }
      spekFrameFlags = 0x00;
      #if defined(FAILSAFE)
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // Valid frame, clear FailSafe counter
      #endif
    } else { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
      uint32_t spekInterval = timer0_overflow_count - spekTimeLast;
      if (spekInterval > 3) {spekFrameFlags = 0; }  //If it has been a while, make the interrupt handler start over. 
    }
  }
}
#endif

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  #if defined(SPEKTRUM)
    readSpektrum();
    if (chan < RC_CHANS) {
      data = rcValue[rcChannel[chan]];
    } else data = 1500;
  #else
    uint8_t oldSREG;
    oldSREG = SREG; cli(); // Let's disable interrupts
    data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
    SREG = oldSREG;        // Let's restore interrupt state
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;

    //Read from raw receiver
    #if defined(SBUS)
      readSBus();
      for (chan = 0; chan < RC_CHANS; chan++)
        rcData[chan] = readRawRC(chan);
    #elif defined(SPEKTRUM)
      for (chan = 0; chan < RC_CHANS; chan++)
        rcData[chan] = readRawRC(chan);
    #else
      rc4ValuesIndex++;
      if (rc4ValuesIndex == 4) rc4ValuesIndex = 0;
      for (chan = 0; chan < RC_CHANS; chan++) 
      {
        #if defined(FAILSAFE)
          uint16_t rcval = readRawRC(chan);
          if(rcval>FAILSAFE_DETECT_TRESHOLD || chan > 3 || !f.ARMED) 
          {        // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
            rcData4Values[chan][rc4ValuesIndex] = rcval;                      // In disarmed state allow always update for easer configuration.
          }
        #else
          rcData4Values[chan][rc4ValuesIndex] = readRawRC(chan);
        #endif
        rcDataMean[chan] = 0;
        for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
        rcDataMean[chan]= (rcDataMean[chan]+2)>>2;
        if ( rcDataMean[chan] < (uint16_t)rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
        if ( rcDataMean[chan] > (uint16_t)rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
      }
    #endif

    //Read from serial link (Bluetooth)
    for (chan = 0; chan < RC_CHANS; chan++) 
    {
      if (chan<8 && rcSerialCount > 0) { // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
        rcSerialCount --;
        #if defined(FAILSAFE)
          failsafeCnt = 0;
        #endif
        if (rcSerial[chan] >900) 
        {
          rcData[chan] = rcSerial[chan];
          
          rcDataCmds[chan] = rcSerial[chan];		 
        } // only relevant channels are overridden
      }
    }
    
    if (danceYAWflag == HIGH)
    {
      rcData[YAW] = desiredDanceYaw;
    }
	static uint16_t prevAUX2 = 1000;
	if (rcData[AUX2] == 1100 && prevAUX2 != 1100 && f.ARMED) //captures that quad was hit by rocket
	{
		hitFlag = HIGH;
	}
	else if (rcData[AUX2] == 1200 && prevAUX2 != 1200 && f.ARMED) //Instructs quad to do a flip
	{
		flipFlag = HIGH;
	}
  else if (rcData[AUX2] == 1300 && prevAUX2 != 1300 && f.ARMED) //should be 1300 
  {
    keepdistFlag = HIGH;
  }
  else if (rcData[AUX2] == 1400 && prevAUX2 != 1400 && f.ARMED)
  {
    chasecarFlag = HIGH;
  }
  else if (rcData[AUX2] == 1600 && prevAUX2 != 1600 && f.ARMED) //1600
  {
    swingFlag = HIGH;
  }
  else if (rcData[AUX2] == 1700 && prevAUX2 != 1700)
 {
    danceYAWflag = HIGH; 
 }
  
  //De-activating a flag for behaviours that don't automatically 
  if ((rcData[AUX2] != 1300) && (prevAUX2 == 1300) && f.ARMED)
  {
    keepdistFlag = LOW;
  }
  else if ((rcData[AUX2] != 1400) && (prevAUX2 == 1400) && f.ARMED)
  {
    chasecarFlag = LOW;
  }

	prevAUX2 = rcData[AUX2];
}
