// in vim, :set ts=2 sts=2 sw=2 et

// EnableInterrupt, a library by GreyGnome.  Copyright 2014-2015 by Michael Anthony Schwager.

/*
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/ 

// Many definitions in /usr/avr/include/avr/io.h

#ifndef EnableInterrupt_h
#define EnableInterrupt_h
#include <Arduino.h>

// *************************************************************************************
// *************************************************************************************
// Function Prototypes *****************************************************************
// *************************************************************************************
// *************************************************************************************

// These are the only functions the end user (programmer) needs to consider. This means you!

// Arduino Due (not Duemilanove) macros. Easy-peasy.
#if defined __SAM3U4E__ || defined __SAM3X8E__ || defined __SAM3X8H__
#ifdef NEEDFORSPEED
#error The NEEDFORSPEED definition does not make sense on the Due platform.
#endif
define enableInterrupt(pin,userFunc,mode) attachInterrupt(pin, userFunc,mode)
define disableInterrupt(pin) detachInterrupt(pin)
#else
/* 
 * enableInterrupt- Sets up an interrupt on a selected Arduino pin.
 * or
 * enableInterruptFast- When used with the NEEDFORSPEED macro, sets up an interrupt on a selected Arduino pin.
 * 
 * Usage:
 * enableInterrupt(uint8_t pinNumber, void (*userFunction)(void), uint8_t mode);
 * or
 * enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);
 *
 * For HiSpeed mode,
 * enableInterruptFast(uint8_t pinNumber, uint8_t mode);
 * or
 * enableInterruptFast(uint8_t interruptDesignator, uint8_t mode);
 *
 * ---------------------------------------------------------------------------------------
 *
 * disableInterrupt- Disables interrupt on a selected Arduino pin.
 *
 * Usage:
 *
 * disableInterrupt(uint8_t pinNumber);
 * or
 * disableInterrupt(uint8_t interruptDesignator);
 *
 * ---------------------------------------------------------------------------------------
 *
 * interruptDesignator: Essentially this is an Arduino pin, and if that's all you want to give
 * the function, it will work just fine. Why is it called an "interruptDesignator", then? Because
 * there's a twist: You can perform a bitwise "and" with the pin number and PINCHANGEINTERRUPT
 * to specify that you want to use a Pin Change Interrupt type of interrupt on those pins that
 * support both Pin Change and External Interrupts. Otherwise, the library will choose whatever
 * interrupt type (External, or Pin Change) normally applies to that pin, with priority to
 * External Interrupt. 
 *
 * Believe it or not, that complexity is all because of pins 2 and 3 on the ATmega328-based
 * Arduinos. Those are the only pins in the Arduino line that can share External or Pin Change
 * Interrupt types. Otherwise, each pin only supports a single type of interrupt and the
 * PINCHANGEINTERRUPT scheme changes nothing. This means you can ignore this whole discussion
 * for ATmega2560- or ATmega32U4-based Arduinos. You can probably safely ignore it for
 * ATmega328-based Arduinos, too.
 */
void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);
void disableInterrupt(uint8_t interruptDesignator);
void bogusFunctionPlaceholder(void);
#ifdef NEEDFORSPEED
#undef enableInterruptFast
// enableInterruptFast(uint8_t interruptDesignator, uint8_t mode);
#define enableInterruptFast(x, y) enableInterrupt(x, bogusFunctionPlaceholder, y)
#endif


// *************************************************************************************
// End Function Prototypes *************************************************************
// *************************************************************************************

#undef PINCHANGEINTERRUPT
#define PINCHANGEINTERRUPT 0x80

#undef attachPinChangeInterrupt
#undef detachPinChangeInterrupt
#define detachPinChangeInterrupt(pin)                   disableInterrupt(pin)
#define attachPinChangeInterrupt(pin,userFunc,mode)     enableInterrupt(pin , userFunc,mode)

#ifndef LIBCALL_ENABLEINTERRUPT // LIBCALL_ENABLEINTERRUPT ****************************************
#ifdef NEEDFORSPEED
void bogusFunctionPlaceholder(void) {
}
#include "pindefs_speed.h"
#endif

// Example: EI_printPSTR("This is a nice long string that takes no static ram");
/*
#define EI_printPSTR(x) SerialPrint_P(PSTR(x))
void SerialPrint_P(const char *str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) Serial.write(c);
} 
*/

/* Arduino pin to ATmega port translaton is found doing digital_pin_to_port_PGM[] */
/* Arduino pin to PCMSKx bitmask is found by doing digital_pin_to_bit_mask_PGM[] */
/* ...except for PortJ, which is shifted left 1 bit in PCI1 */
volatile uint8_t *pcmsk;

// Arduino.h has these, but the block is surrounded by #ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#define TOTAL_PORTS 13 // The 12 above, plus 0 which is not used.

typedef void (*interruptFunctionType)(void);

// ===========================================================================================
// CHIP SPECIFIC DATA STRUCTURES =============================================================
// ===========================================================================================

/* UNO SERIES *************************************************************************/
/* UNO SERIES *************************************************************************/
/* UNO SERIES *************************************************************************/
#if defined __AVR_ATmega168__ || defined __AVR_ATmega168A__ || defined __AVR_ATmega168P__ || \
  __AVR_ATmega168PA__ || \
  __AVR_ATmega328__ || __AVR_ATmega328P__

#define ARDUINO_328

#ifndef NEEDFORSPEED
const uint8_t PROGMEM digital_pin_to_port_bit_number_PGM[] = {
  0, // 0 == port D, 0
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  0, // 8 == port B, 0
  1,
  2,
  3,
  4,
  5,
  0, // 14 == port C, 0
  1,
  2,
  3,
  4,
  5,
};

interruptFunctionType functionPointerArrayEXTERNAL[2];
// 2 of the interrupts are unsupported on Arduino UNO.
struct functionPointersPortB {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
};
typedef struct functionPointersPortB functionPointersPortB;

functionPointersPortB portBFunctions = { NULL, NULL, NULL, NULL, NULL, NULL };

// 1 of the interrupts are used as RESET on Arduino UNO.
struct functionPointersPortC {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
};
typedef struct functionPointersPortC functionPointersPortC;

functionPointersPortC portCFunctions = { NULL, NULL, NULL, NULL, NULL, NULL };

// 1 of the interrupts are used as RESET on Arduino UNO.
struct functionPointersPortD {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
  interruptFunctionType pinSix;
  interruptFunctionType pinSeven;
};
typedef struct functionPointersPortD functionPointersPortD;

functionPointersPortD portDFunctions = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
#endif // NEEDFORSPEED

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how the ports were defined.
volatile uint8_t risingPinsPORTB=0;
volatile uint8_t fallingPinsPORTB=0;
volatile uint8_t risingPinsPORTC=0;
volatile uint8_t fallingPinsPORTC=0;
volatile uint8_t risingPinsPORTD=0;
volatile uint8_t fallingPinsPORTD=0;

// for the saved state of the ports
static volatile uint8_t portSnapshotB;
static volatile uint8_t portSnapshotC;
static volatile uint8_t portSnapshotD;

// these are defined in the avr.h files, like iom328p.h
#define PORTB_VECT PCINT0_vect
#define PORTC_VECT PCINT1_vect
#define PORTD_VECT PCINT2_vect

#endif // #if defined __AVR_ATmega168__ || defined __AVR_ATmega168A__ ...

// ===========================================================================================
// END END END DATA STRUCTURES ===============================================================
// ===========================================================================================

// From /usr/share/arduino/hardware/arduino/cores/robot/Arduino.h
// #define CHANGE 1
// #define FALLING 2
// #define RISING 3

// "interruptDesignator" is simply the Arduino pin optionally OR'ed with
// PINCHANGEINTERRUPT (== 0x80)
void enableInterrupt(uint8_t interruptDesignator, interruptFunctionType userFunction, uint8_t mode) 
{
  uint8_t arduinoPin;
  uint8_t portNumber=0;
  uint8_t portMask=0;
#ifndef NEEDFORSPEED
  uint8_t portBitNumber; // when an interrupted pin is found, this will be used to choose the function.
  interruptFunctionType *calculatedPointer;
#endif

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;

  // *************************************************************************************
  // *************************************************************************************
  // Pin Change Interrupts
  // *************************************************************************************
  // *************************************************************************************
#if defined ARDUINO_328
  if ( (interruptDesignator & PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
#else
#error Unsupported Arduino platform
#endif
    {
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]);
      portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);
    }

    // save the mode
    if ((mode == RISING) || (mode == CHANGE)) {
      if (portNumber==PB) {
        risingPinsPORTB |= portMask;
      }
#if defined ARDUINO_328
      if (portNumber==PC) {
        risingPinsPORTC |= portMask;
      }
      if (portNumber==PD) {
        risingPinsPORTD |= portMask;
      }
#endif
    }
    if ((mode == FALLING) || (mode == CHANGE)) {
      if (portNumber==PB) {
        fallingPinsPORTB |= portMask;
      }
#if defined ARDUINO_328
      if (portNumber==PC) {
        fallingPinsPORTC |= portMask;
      }
      if (portNumber==PD) {
        fallingPinsPORTD |= portMask;
      }
#endif
    }

#ifndef NEEDFORSPEED
    // assign the function to be run in the ISR
    // save the initial value of the port
    portBitNumber=pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin]);
#endif
#if defined ARDUINO_328
    if (portNumber==PC) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portCFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotC=*portInputRegister(portNumber);
      pcmsk=&PCMSK1;
      PCICR |= _BV(1);
    }
    if (portNumber==PD) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portDFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotD=*portInputRegister(portNumber);
      pcmsk=&PCMSK2;
      PCICR |= _BV(2);
    }

#endif // defined ARDUINO_328
    if (portNumber==PB) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portBFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotB=*portInputRegister(portNumber);
      pcmsk=&PCMSK0;
      PCICR |= _BV(0);
    }
    *pcmsk |= portMask;  // appropriate bit, e.g. this could be PCMSK1 |= portMask;

    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. GIE may have already been set on a previous enable, so it's important
    // to take note of the order in which things were done, above.

  // *************************************************************************************
  // *************************************************************************************
  // External Interrupts
  // *************************************************************************************
  // *************************************************************************************
  } else {
    uint8_t origSREG; // to save for interrupts
    origSREG = SREG;
    cli(); // no interrupts while we're setting up an interrupt.
#if defined ARDUINO_328
    if (arduinoPin == 3) {
#ifndef NEEDFORSPEED
      functionPointerArrayEXTERNAL[1] = userFunction;
#endif
      EIMSK &= ~_BV(1);
      EICRA &= (~_BV(2) & ~_BV(3));
      EICRA |= mode << 2;
      EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
      EIMSK |= _BV(1);
    } else {
#ifndef NEEDFORSPEED
      functionPointerArrayEXTERNAL[0] = userFunction;
#endif
      EIMSK &= ~_BV(0);
      EICRA &= (~_BV(0) & ~_BV(1));
      EICRA |= mode;
      EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
      EIMSK |= _BV(0);
    }
#endif
    SREG=origSREG;
  }
  SREG |= (1 << SREG_I); // GIE bit in SREG. From /usr/avr/include/avr/common.h
}

void disableInterrupt (uint8_t interruptDesignator) {

  uint8_t origSREG; // to save for interrupts
  uint8_t arduinoPin;
  uint8_t portNumber=0;
  uint8_t portMask=0;

  origSREG = SREG;
  cli();
  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;
#if defined ARDUINO_328
  if ( (interruptDesignator & PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
#else
#error Unsupported Arduino platform
#endif
    {
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]);
      portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);
    }
    if (portNumber == PB) {
      PCMSK0 &= ~portMask;
      if (PCMSK0 == 0) { PCICR &= ~_BV(0); };
      risingPinsPORTB &= ~portMask;
      fallingPinsPORTB &= ~portMask;
    }
#if defined ARDUINO_328
    if (portNumber == PC) {
      PCMSK1 &= ~portMask;
      if (PCMSK1 == 0) { PCICR &= ~_BV(1); };
      risingPinsPORTC &= ~portMask;
      fallingPinsPORTC &= ~portMask;
    }
    if (portNumber == PD) {
      PCMSK2 &= ~portMask;
      if (PCMSK2 == 0) { PCICR &= ~_BV(2); };
      risingPinsPORTD &= ~portMask;
      fallingPinsPORTD &= ~portMask;
    }
#endif
  } else {
#if defined ARDUINO_328
    if (arduinoPin == 3) {
      EIMSK &= ~_BV(1);
      EICRA &= (~_BV(2) & ~_BV(3));
      EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
    } else {
      EIMSK &= ~_BV(0);
      EICRA &= (~_BV(0) & ~_BV(1));
      EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
    }
#endif
  }
  SREG = origSREG;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
////////////////////// ISRs /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

/*
ISR(INT0_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[0])();
#else
#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PIN21
  INTERRUPT_FLAG_PIN21++;
#endif
#endif
#if defined ARDUINO_LEONARDO
#ifdef INTERRUPT_FLAG_PIN3
  INTERRUPT_FLAG_PIN3++;
#endif
#endif
#if defined ARDUINO_328
#ifdef INTERRUPT_FLAG_PIN2
  INTERRUPT_FLAG_PIN2++;
#endif
#endif
#endif // NEEDFORSPEED
}
*/
/*

ISR(INT1_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[1])();
#else
#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PIN20
  INTERRUPT_FLAG_PIN20++;
#endif
#endif
#if defined ARDUINO_LEONARDO
#ifdef INTERRUPT_FLAG_PIN2
  INTERRUPT_FLAG_PIN2++;
#endif
#endif
#if defined ARDUINO_328
#ifdef INTERRUPT_FLAG_PIN3
  INTERRUPT_FLAG_PIN3++;
#endif
#endif
#endif // NEEDFORSPEED
}

*/






ISR(PORTB_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PINB;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotB ^ current;
  tmp           = risingPinsPORTB & current;
  interruptMask = fallingPinsPORTB & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK0 & interruptMask;


  portSnapshotB = current;
#ifdef NEEDFORSPEED
#include "portb_speed.h"
#else
  if (interruptMask == 0) goto exitPORTBISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portBFunctions.pinZero();
  if (interruptMask & _BV(1)) portBFunctions.pinOne();
  if (interruptMask & _BV(2)) portBFunctions.pinTwo();
  if (interruptMask & _BV(3)) portBFunctions.pinThree();
  if (interruptMask & _BV(4)) portBFunctions.pinFour();
  if (interruptMask & _BV(5)) portBFunctions.pinFive();
#ifndef ARDUINO_328
  if (interruptMask & _BV(6)) portBFunctions.pinSix();
  if (interruptMask & _BV(7)) portBFunctions.pinSeven();
#endif
  exitPORTBISR: return;
  // FOR MEASUREMENT ONLY
  // exitPORTBISR: PORTC &= ~(1 << PC5); // SIGNAL THAT WE ARE LEAVING THE INTERRUPT
#endif // NEEDFORSPEED
}


#if defined ARDUINO_328

ISR(PORTC_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PINC;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotC ^ current;
  tmp           = risingPinsPORTC & current;
  interruptMask = fallingPinsPORTC & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK1 & interruptMask;

  portSnapshotC = current;
#ifdef NEEDFORSPEED
#include "portc_speed.h"
#else
  if (interruptMask == 0) goto exitPORTCISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portCFunctions.pinZero();
  if (interruptMask & _BV(1)) portCFunctions.pinOne();
  if (interruptMask & _BV(2)) portCFunctions.pinTwo();
  if (interruptMask & _BV(3)) portCFunctions.pinThree();
  if (interruptMask & _BV(4)) portCFunctions.pinFour();
  if (interruptMask & _BV(5)) portCFunctions.pinFive();
  exitPORTCISR: return;
#endif // NEEDFORSPEED
}



ISR(PORTD_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PIND;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotD ^ current;
  tmp           = risingPinsPORTD & current;
  interruptMask = fallingPinsPORTD & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK2 & interruptMask;


  portSnapshotD = current;
#ifdef NEEDFORSPEED
#include "portd_speed.h"
#else
  if (interruptMask == 0) goto exitPORTDISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portDFunctions.pinZero();
  if (interruptMask & _BV(1)) portDFunctions.pinOne();
  if (interruptMask & _BV(2)) portDFunctions.pinTwo();
  if (interruptMask & _BV(3)) portDFunctions.pinThree();
  if (interruptMask & _BV(4)) portDFunctions.pinFour();
  if (interruptMask & _BV(5)) portDFunctions.pinFive();
  if (interruptMask & _BV(6)) portDFunctions.pinSix();
  if (interruptMask & _BV(7)) portDFunctions.pinSeven();
  exitPORTDISR: return;
#endif // NEEDFORSPEED
}




#endif // defined ARDUINO_328

#endif // #ifndef LIBCALL_ENABLEINTERRUPT *********************************************************
#endif // #if defined __SAM3U4E__ || defined __SAM3X8E__ || defined __SAM3X8H__
#endif // #ifndef EnableInterrupt_h ***************************************************************
