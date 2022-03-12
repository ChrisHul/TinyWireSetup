//
// Bit Bang I2C library
// Copyright (c) 2018-2019 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 10/12/2018
//
// Modified by ChrHul 10/03/2022
// Implementation of SCL stretching, so that slave can stop the transfer
// for internal processing
// New directive for optional including Wire library to reduce code
// Cutting in half delays to make bitrate more accurate
// In the process of debugging slave software much of the code has been
// changed many times with no other intention than testing the slave
// and enhance readability
// Therefore the appearance of some sections may have changed.
// The modifications have been tested on Atmega328 only.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//#define INCLUDE_WIRE

#ifdef _LINUX_
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <pigpio.h>
#define PROGMEM
#define false 0
#define true 1
#define memcpy_P memcpy
#define INPUT 1
#define OUTPUT 2

// maps RPI pins to BCM GPIO numbers
const int iRPIPins[] = {-1,-1,-1,2,-1,3,-1,4,14,-1,
                        15,17,18,27,-1,22,23,-1,24,10,
                        -1,9,25,11,8,-1,7,0,1,5,
                        -1,6,12,13,-1,19,16,26,20,-1,
                        21};

#else // Arduino
#include <Arduino.h>
#ifndef __AVR_ATtiny85__
#include <Wire.h>
#endif
#ifdef W600_EV
#include <W600FastIO.h>
#define VARIANT_MCK 80000000ul
#endif
#endif // _LINUX_
#include <BitBang_I2C.h>
volatile static uint8_t iSCL, iSDA; // keep requested pin numbers in private statics
volatile static int iDelay; // bit delay in ms for the requested clock rate
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
volatile static uint8_t *iDDR_SCL, *iPort_SCL_In, *iPort_SCL_Out;
volatile static uint8_t *iDDR_SDA, *iPort_SDA_In, *iPort_SDA_Out;
volatile static uint8_t iSDABit, iSCLBit;
#endif
#ifdef FUTURE
//#else // must be a 32-bit MCU
volatile uint32_t *iDDR_SCL, *iPort_SCL_In, *iPort_SCL_Out;
volatile uint32_t *iDDR_SDA, *iPort_SDA_In, *iPort_SDA_Out;
uint32_t iSDABit, iSCLBit;
#endif

#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
uint8_t getPinInfo(uint8_t pin, volatile uint8_t **iDDR, volatile uint8_t **iPort, int bInput)
{
  uint8_t port, bit;

  port = (pin & 0xf0); // hex port (A,B,D,E,F)
  bit = pin & 0x7;
  switch (port)
  {
#ifdef PORTE
    case 0xE0:
      *iPort = (bInput) ? &PINE : &PORTE;
      *iDDR = &DDRE;
      break;
#endif
#ifdef PORTF
    case 0xF0:
      *iPort = (bInput) ? &PINF : &PORTF;
      *iDDR = &DDRF;
      break;
#endif
#ifdef PORTG
    case 0xA0: // really port G
      *iPort = (bInput) ? &PING : &PORTG;
      *iDDR = &DDRG;
      break;
#endif
#ifdef PORTC
    case 0xC0:
      *iPort = (bInput) ? &PINC : &PORTC;
      *iDDR = &DDRC;
      break;
#endif
#ifdef PORTB
    case 0xB0:
      *iPort = (bInput) ? &PINB : &PORTB;
      *iDDR = &DDRB;
      break;
#endif
#ifdef PORTD
    case 0xD0:
      *iPort = (bInput) ? &PIND : &PORTD;
      *iDDR = &DDRD;
      break;
#endif
  }
  return bit;
} /* getPinInfo() */
#endif

//#else // 32-bit version
#ifdef FUTURE
uint32_t getPinInfo(uint8_t pin, volatile uint32_t **iDDR, volatile uint32_t **iPort, int bInput)
{
  uint32_t port, bit;

  if (pin <= 0xbf) // port 0
  {
    *iPort = (bInput) ? &REG_PORT_IN0 : &REG_PORT_OUT0;
    *iDDR = &REG_PORT_DIR0;
  }
  else if (pin <= 0xdf) // port 1
  {
    *iPort = (bInput) ? &REG_PORT_IN1 : &REG_PORT_OUT1;
    *iDDR = &REG_PORT_DIR1;
  }
  else return 0xffffffff; // invalid
  bit = pin & 0x1f;
  return bit;
} /* getPinInfo() */
#endif // __AVR__

inline uint8_t SCL_READ( )
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSCL >= 0xa0) // direct pin numbering
  {
    if (*iPort_SCL_In & iSCLBit)
       return HIGH;
    else
       return LOW;
  }
  else
  {
    return digitalRead(iSCL);
  }
#else
#ifdef __AVR_ATtiny85__
#else
#ifdef W600_EV
    return w600DigitalRead(iSCL);
#else
#ifdef _LINUX_
    return gpioRead(iSCL);
#else
    return digitalRead(iSCL);
#endif // _LINUX_
#endif // W600_EV
#endif // __AVR_ATtiny85__
#endif // ( __AVR__ ) && !( ARDUINO_ARCH_MEGAAVR )
}

inline uint8_t SDA_READ( )
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSDA >= 0xa0) // direct pin numbering
  {
    if (*iPort_SDA_In & iSDABit)
       return HIGH;
    else
       return LOW;
  }
  else
  {
    return digitalRead(iSDA);
  }
#else
#ifdef __AVR_ATtiny85__
#else
#ifdef W600_EV
    return w600DigitalRead(iSDA);
#else
#ifdef _LINUX_
    return gpioRead(iSDA);
#else
    return digitalRead(iSDA);
#endif // _LINUX_
#endif // W600_EV
#endif // __AVR_ATtiny85__
#endif // ( __AVR__ ) && !( ARDUINO_ARCH_MEGAAVR )
}

inline void SCL_HIGH( )
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSCL >= 0xa0) // direct pin numbering
  {
    *iDDR_SCL &= ~iSCLBit;
  }
  else
  {
    pinMode(iSCL, INPUT);
  }  
#else
#ifdef __AVR_ATtiny85__
#else
#ifdef W600_EV
    w600PinMode(iSCL, GPIO_INPUT);
#else
#ifdef _LINUX_
    gpioSetMode(iSCL, PI_INPUT);
#else
    pinMode(iSCL, INPUT);
#endif // _LINUX_
#endif // W600_EV
#endif // __AVR_ATtiny85__
#endif // ( __AVR__ ) && !( ARDUINO_ARCH_MEGAAVR )
}

inline void SCL_LOW( )
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSCL >= 0xa0) // direct pin numbering
  {
    *iDDR_SCL |= iSCLBit;
  }
  else
  {
    pinMode(iSCL, OUTPUT);
  }  
#else
#ifdef __AVR_ATtiny85__
#else
#ifdef W600_EV
    w600PinMode(iSCL, GPIO_OUTPUT);
    w600DigitalWrite(iSCL, LOW);
#else
#ifdef _LINUX_
    gpioSetMode(iSCL, PI_OUTPUT);
#else
    pinMode(iSCL, OUTPUT);
#endif // _LINUX_
#endif // W600_EV
#endif // __AVR_ATtiny85__
#endif // ( __AVR__ ) && !( ARDUINO_ARCH_MEGAAVR )
}

inline void SDA_HIGH( )
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSDA >= 0xa0) // direct pin numbering
  {
    *iDDR_SDA &= ~iSDABit;
  }
  else
  {
    pinMode(iSDA, INPUT);
  }  
#else
#ifdef __AVR_ATtiny85__
#else
#ifdef W600_EV
    w600PinMode(iSDA, GPIO_INPUT);
#else
#ifdef _LINUX_
    gpioSetMode(iSDA, PI_INPUT);
#else
    pinMode(iSDA, INPUT);
#endif // _LINUX_
#endif // W600_EV
#endif // __AVR_ATtiny85__
#endif // ( __AVR__ ) && !( ARDUINO_ARCH_MEGAAVR )
}

inline void SDA_LOW( )
{
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
  if (iSDA >= 0xa0) // direct pin numbering
  {
    *iDDR_SDA |= iSDABit;
  }
  else
  {
    pinMode(iSDA, OUTPUT);
  }  
#else
#ifdef __AVR_ATtiny85__
#else
#ifdef W600_EV
    w600PinMode(iSDA, GPIO_OUTPUT);
    w600DigitalWrite(iSDA, LOW);
#else
#ifdef _LINUX_
    gpioSetMode(iSDA, PI_OUTPUT);
#else
    pinMode(iSDA, OUTPUT);
#endif // _LINUX_
#endif // W600_EV
#endif // __AVR_ATtiny85__
#endif // ( __AVR__ ) && !( ARDUINO_ARCH_MEGAAVR )
}


void inline my_sleep_us(int iDelay)
{
#ifdef __AVR_ATtiny85__
  iDelay *= 2;
  while (iDelay)
  {
    __asm__ __volatile__ (
    "nop" "\n\t"
    "nop"); //just waiting 2 cycle
    iDelay--;
  }
#else
  if (iDelay > 0)
#ifdef _LINUX_
     gpioDelay(iDelay);
#else
     delayMicroseconds(iDelay);
#endif // _LINUX
#endif
} /* my_sleep_us() */

// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//

static uint8_t i2cByteOut(uint8_t b)
{
uint8_t i;
uint16_t c;
uint8_t ack;

  for (i=0; i<8; i++)
  {
      if (b & 0x80)
        SDA_HIGH(); // set data line to 1
      else
        SDA_LOW(); // set data line to 0
      my_sleep_us(iDelay); // let SDA settle
      SCL_HIGH(); // clock high (slave latches data)
      c = 0; while( SCL_READ() == LOW)
              {if( c++ > 2000) return 0;} // clock stretch
      my_sleep_us(iDelay);
      b <<=  1;
      SCL_LOW();

  }

  SDA_HIGH(); // release data line for reading
  my_sleep_us(6*iDelay); // tweak to make work at higher bitrates
  SCL_HIGH();
  c = 0; while( SCL_READ() == LOW)
          {if( c++ > 2000) return 0;} //  clock stretch
  my_sleep_us(iDelay);
  ack = SDA_READ();
  SCL_LOW();
  my_sleep_us(iDelay); // make sure SCL stays low min time
  return (ack == 0) ? 1:0; // a low ACK bit means success
} // i2cByteOut() 

// following fast operations dubious
// SDA and SCL have to be on same PORT
// changing both lines simultaneously may trigger
// unwanted START/STOP events and yield unreliable
// operation
#define BOTH_LOW_FAST *iDDR = both_low;
#define BOTH_HIGH_FAST *iDDR = both_high;
#define SCL_HIGH_FAST *iDDR = scl_high; 
#define SDA_HIGH_FAST *iDDR = sda_high;
#define SDA_READ_FAST *iDDR & iSDABit
#define SCL_READ_FAST *iDDR & iSCLBit

static uint8_t i2cByteOutAVRFast( uint8_t b)
{
uint8_t i, ack;

#define iDDR iDDR_SDA
uint8_t bOld = *iDDR; // current value
uint8_t both_low = bOld | iSDABit | iSCLBit;
uint8_t both_high = bOld & ~(iSDABit | iSCLBit);
uint8_t scl_high = (bOld | iSDABit) & ~iSCLBit;
uint8_t sda_high = (bOld | iSCLBit) & ~iSDABit;
uint16_t c;

     BOTH_LOW_FAST // start with both lines set to 0
     for (i=0; i<8; i++)
     {
         if (b & 0x80)
         {
           SDA_HIGH_FAST // set data line to 1
           my_sleep_us(iDelay);
           BOTH_HIGH_FAST // rising edge clocks data
         }
         else // more probable case (0) = shortest code path
         {
           SCL_HIGH_FAST // clock high (slave latches data)
         }
         c = 0; while( SCL_READ() == 0)
              {if( c++ > 2000) return 0;}
         my_sleep_us(iDelay);
         BOTH_LOW_FAST // should be avoided - can trigger stop/start signal
         b <<= 1;
     } // for i
// read ack bit
  SDA_HIGH_FAST // set data line for reading
  BOTH_HIGH_FAST // clock line high
  my_sleep_us(iDelay); // DEBUG - delay/2
  ack = SDA_READ_FAST;
  BOTH_LOW_FAST // should be avoided - can trigger start signal
  c = 0; while( SCL_READ() == 0)  // SCL stretch
          {if( c++ > 2000) return 0;} 
  my_sleep_us(iDelay); // DEBUG - delay/2
  return (ack == 0) ? 1:0; // a low ACK bit means success
} /* i2cByteOutAVRFast() */

//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
static uint8_t i2cByteIn( uint8_t bLast)
{
uint8_t i;
uint16_t c;
uint8_t b = 0xFF;

     SDA_HIGH(); // set data line as input
     for (i=0; i<8; i++)
     {
         my_sleep_us(iDelay); // wait for SDA to settle
         SCL_HIGH();
         c = 0; while( SCL_READ() == LOW)
               {if( c++ > 2000) return 0x13;} // SCL stretch
         my_sleep_us(iDelay);
         b <<= 1;
         if ( SDA_READ() != LOW) // read the data bit
              b |= 1; // set data bit
         SCL_LOW();
     } // for i
     if (bLast == 0)
        SDA_LOW(); // send ACK if not last byte - otherwise NACK
     my_sleep_us(iDelay); // wait for SDA to settle
     SCL_HIGH(); // clock high to send ack/nack
     c = 0; while( SCL_READ() == LOW) // SCL stretch
          {if( c++ > 2000) return 0x12;} // todo: some error indication
     my_sleep_us(iDelay);
     SCL_LOW( );
     my_sleep_us(iDelay);

     return b;
} /* i2cByteIn() */

//
// Send I2C STOP condition
//
static void i2cEnd()
{
   uint16_t c = 0;
   // first make sure SDA is released
   SDA_HIGH(); // data high
   while( SDA_READ() == LOW) {
       // if SDA blocked try to clock away
       SCL_HIGH(); my_sleep_us(iDelay);
       SCL_LOW(); my_sleep_us(iDelay);
       if( c++ > 20) return;
   }
   SDA_LOW(); // data line low
   my_sleep_us(iDelay);
   SCL_HIGH(); // clock high
   while( SCL_READ() == LOW)
     {if(c++ > 2000) break;} // make sure it goes high
   delayMicroseconds(1);
   SDA_HIGH(); // data high with little delay
   my_sleep_us(iDelay);
} /* i2cEnd() */


static int i2cBegin(I2Chandle_t *pI2C, uint8_t addr, uint8_t bRead)
{

iSDA = pI2C->iSDA;
iSCL = pI2C->iSCL;
iDDR_SDA = pI2C->iDDR_SDA;
iDDR_SCL = pI2C->iDDR_SCL;
iSDABit = pI2C->iSDABit;
iSCLBit = pI2C->iSCLBit;
iPort_SDA_In = pI2C->iPort_SDA_In;
iPort_SCL_In = pI2C->iPort_SCL_In;
iPort_SDA_Out = pI2C->iPort_SDA_Out;
iPort_SCL_Out = pI2C->iPort_SCL_Out;
iDelay = pI2C->iDelay;

   int rc;
   uint8_t c = 0;
   SDA_HIGH(); // release data line
   while( SDA_READ() == LOW) {
       // if SDA blocked try to clock away
       SCL_HIGH(); my_sleep_us(iDelay);
       SCL_LOW(); my_sleep_us(iDelay);
       if( c++ > 20) return 0;
   }
   my_sleep_us(iDelay);
   SCL_HIGH();
   while( SCL_READ() == LOW)
       {if(c++ > 2000) return 0;} // make sure it goes high
   my_sleep_us(iDelay);

// here both SDA and SCL should be high
   SDA_LOW(); // send data line low first
   my_sleep_us(iDelay);
   SCL_LOW(); // then clock line low
   my_sleep_us(iDelay);
   addr <<= 1;
   if ( bRead) addr++; // set read bit
#ifdef __AVR_ATtiny85__
   rc = i2cByteOutAVR(addr); // not implemented
#else
   rc = i2cByteOut(addr); // send the slave address and R/W bit
#endif
   my_sleep_us(iDelay);

   return rc;
} /* i2cBegin() */


static inline int i2cWrite( uint8_t *pData, int iLen)
{
uint8_t b;
int rc, iOldLen = iLen;

   rc = 1;
   while (iLen && rc == 1)
   {
      b = *pData++;
#ifdef __AVR_ATtiny85__
      rc = i2cByteOutAVRFast(b);
#else
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
     if (iSDA >= 0xa0)
     {
           rc = i2cByteOut( b);
     }
     else
#endif
     {
// can't use direct manipulation, because port and pin not known if iSDA < 0xa0.
/*        if (b == 0xff || b == 0)
           rc = i2cByteOutFast( b); // speed it up a bit more if all bits are ==
        else*/
           rc = i2cByteOut( b);
     }
#endif
      if (rc == 1) // success
      {
         iLen--;
      }
   } // for each byte
   return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
} /* i2cWrite() */

static inline void i2cRead( uint8_t *pData, int iLen)
{
   while (iLen--)
   {
      *pData++ = i2cByteIn( iLen == 0);
   } // for each byte
} /* i2cRead() */
//
// Initialize the I2C BitBang library
// Pass the pin numbers used for SDA and SCL
// as well as the clock rate in Hz
//
void I2CInit( I2Chandle_t *pI2C, uint32_t iClock)
{
#ifdef _LINUX_
   if (gpioInitialise() < 0)
   {
      printf("pigpio failed to initialize\n");
      return;
   }
#endif
   if (pI2C == NULL) return;

#if defined INCLUDE_WIRE
   if (pI2C->bWire) // use Wire library
   {
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
#if defined(TEENSYDUINO) || defined(ARDUINO_ARCH_RP2040) || defined( __AVR__ ) || defined( NRF52 ) || defined ( ARDUINO_ARCH_NRF52840 ) || defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_SAM)
       Wire.begin();
#else
       if (pI2C->iSDA == 0xff || pI2C->iSCL == 0xff)
          Wire.begin();
       else
          Wire.begin(pI2C->iSDA, pI2C->iSCL);
#endif
       Wire.setClock(iClock);
#endif //!defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
#ifdef _LINUX_
       {
           char filename[32];
           sprintf(filename, "/dev/i2c-%d", pI2C->iBus);
           if ((pI2C->file_i2c = open(filename, O_RDWR)) < 0)
                 return;
       }
#endif // _LINUX_
       return;
   }
#endif // INCLUDE_WIRE
   if (pI2C->iSDA < 0xa0)
   {
#if !defined( _LINUX_ ) && !defined ( __AVR__ )
#ifdef W600_EV
     w600PinMode(pI2C->iSDA, GPIO_OUTPUT);
     w600PinMode(pI2C->iSCL, GPIO_OUTPUT);
     w600DigitalWrite(pI2C->iSDA, LOW); // setting low = enabling as outputs
     w600DigitalWrite(pI2C->iSCL, LOW);
     w600PinMode(pI2C->iSDA, GPIO_INPUT); // let the lines float (tri-state)
     w600PinMode(pI2C->iSCL, GPIO_INPUT);
#else // generic
     pinMode(pI2C->iSDA, OUTPUT);
     pinMode(pI2C->iSCL, OUTPUT);
     digitalWrite(pI2C->iSDA, LOW); // setting low = enabling as outputs
     digitalWrite(pI2C->iSCL, LOW);
     pinMode(pI2C->iSDA, INPUT); // let the lines float (tri-state)
     pinMode(pI2C->iSCL, INPUT);
#endif
#endif
#ifdef _LINUX_
     // use PIGPIO
     // convert pin numbers to BCM numbers for PIGPIO
     pI2C->iSDA = iRPIPins[pI2C->iSDA];
     pI2C->iSCL = iRPIPins[pI2C->iSCL];
     gpioWrite(pI2C->iSDA, 0);
     gpioWrite(pI2C->iSCL, 0);
     gpioSetMode(pI2C->iSDA, PI_INPUT);
//     gpioSetPullUpDown(pI2C->iSDA, PI_PUD_UP);
     gpioSetMode(pI2C->iSCL, PI_INPUT);
//     gpioSetPullUpDown(pI2C->iSCL, PI_PUD_UP);
#endif // _LINUX_
   }
#if defined ( __AVR__ ) && !defined( ARDUINO_ARCH_MEGAAVR )
   else // direct pin mode, get port address and bit
   {
     if( pI2C->iSDA >= 0xa0)
     {
        getPinInfo(pI2C->iSDA, &pI2C->iDDR_SDA, &pI2C->iPort_SDA_In, 1);
        pI2C->iSDABit = 1 << getPinInfo(pI2C->iSDA, &pI2C->iDDR_SDA, &pI2C->iPort_SDA_Out, 0);
        getPinInfo(pI2C->iSCL, &pI2C->iDDR_SCL, &pI2C->iPort_SCL_In, 1);
        pI2C->iSCLBit = 1 << getPinInfo(pI2C->iSCL, &pI2C->iDDR_SCL, &pI2C->iPort_SCL_Out, 0);
 
        *(pI2C->iDDR_SDA) &= ~iSDABit; // pinMode input
        *(pI2C->iDDR_SCL) &= ~iSCLBit; // pinMode input
        *(pI2C->iPort_SDA_Out) &= ~iSDABit; // digitalWrite SDA LOW
        *(pI2C->iPort_SCL_Out) &= ~iSCLBit; // digitalWrite SCL LOW
     } else {
        pinMode( pI2C->iSDA, INPUT); // will make them go high with pullup
        pinMode( pI2C->iSCL, INPUT);
	digitalWrite( pI2C->iSDA, LOW);
	digitalWrite( pI2C->iSCL, LOW);
     }
   }
#endif // __AVR__

#ifdef _LINUX_
   pI2C->iDelay = 1000000 / iClock;
   if (pI2C->iDelay < 1) pI2C->iDelay = 1;
#else
   pI2C->iDelay = (uint16_t)(500000 / iClock);
#endif // _LINUX_
} /* i2cInit() */
//
// Test a specific I2C address to see if a device responds
// returns 0 for no response, 1 for a response
//
uint8_t I2CTest(I2Chandle_t *pI2C, uint8_t addr)
{
uint8_t response = 0;

  if (pI2C->bWire)
  {
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
  // We use the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(addr);
  response = !Wire.endTransmission();
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, addr) >= 0)
        response = 1;
#endif
    return response;
  }
  if (i2cBegin(pI2C, addr, 0)) // try to write to the given address
  {
    response = 1;
  }
  i2cEnd();
  return response;
} /* I2CTest() */
//
// Scans for I2C devices on the bus
// returns a bitmap of devices which are present (128 bits = 16 bytes, LSB first)
// A set bit indicates that a device responded at that address
//
void I2CScan(I2Chandle_t *pI2C, uint8_t *pMap)
{
  int i;
  for (i=0; i<16; i++) // clear the bitmap
    pMap[i] = 0;
  for (i=1; i<128; i++) // try every address
  {
    if (I2CTest(pI2C, i))
    {
      pMap[i >> 3] |= (1 << (i & 7));
    }
  }
} /* I2CScan() */
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(I2Chandle_t *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
  int rc = 0;
  
#ifdef INCLUDE_WIRE
  if (pI2C->bWire)
  {
#if !defined ( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
    Wire.beginTransmission(iAddr);
    Wire.write(pData, (uint8_t)iLen);
    rc = !Wire.endTransmission();
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr) >= 0)
    {
       if (write(pI2C->file_i2c, pData, iLen) >= 0)
          rc = 1;
    } 
#endif // _LINUX_
    return rc;
  }
#endif // INCLUDE_WIRE
  rc = i2cBegin(pI2C, iAddr, 0);
  if (rc == 1) // slave sent ACK for its address
  {
     rc = i2cWrite( pData, iLen);
  }
  i2cEnd();
  return rc; // returns the number of bytes sent or 0 for error
} /* I2CWrite() */
//
// Read N bytes starting at a specific I2C internal register
//
int I2CReadRegister(I2Chandle_t *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
  int rc;
  
#ifdef INCLUDE_WIRE
  if (pI2C->bWire) // use the wire library
  {
      int i = 0;
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
      Wire.beginTransmission(iAddr);
      Wire.write(u8Register);
      Wire.endTransmission();
      Wire.requestFrom(iAddr, (uint8_t)iLen);
      while (i < iLen)
      {
          pData[i++] = Wire.read();
      }
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr) >= 0)
    {
       write(pI2C->file_i2c, &u8Register, 1);
       i = read(pI2C->file_i2c, pData, iLen);
    } 
#endif // _LINUX_
      return (i > 0);
  }
#endif // INCLUDE_WIRE

  rc = i2cBegin( pI2C, iAddr, 0); // start a write operation
  if (rc == 1) // slave sent ACK for its address
  {
     rc = i2cWrite( &u8Register, 1); // write the register we want to read from
     if (rc == 1)
     {
       i2cEnd();
       rc = i2cBegin(pI2C, iAddr, 1); // start a read operation
       if (rc == 1)
       {
         i2cRead( pData, iLen);
       }
     }
  }
  i2cEnd();
  return rc; // returns 1 for success, 0 for error
} /* I2CReadRegister() */
//
// Read N bytes
//
int I2CRead(I2Chandle_t *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
  int rc;
  
#ifdef INCLUDE_WIRE
    if (pI2C->bWire) // use the wire library
    {
        int i = 0;
#if !defined( _LINUX_ ) && !defined( __AVR_ATtiny85__ )
        Wire.requestFrom(iAddr, (uint8_t)iLen);
        while (i < iLen)
        {
            pData[i++] = Wire.read();
        }
#endif
#ifdef _LINUX_
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr) >= 0)
    {
       i = read(pI2C->file_i2c, pData, iLen);
    } 
#endif // _LINUX_
        return (i > 0);
    }
#endif // INCLUDE_WIRE
  rc = i2cBegin(pI2C, iAddr, 1);
  if (rc == 1) // slave sent ACK for its address
  {
     i2cRead( pData, iLen);
  }
  i2cEnd();
  return rc; // returns 1 for success, 0 for error
} /* I2CRead() */
//
// Figure out what device is at that address
// returns the enumerated value
//
int I2CDiscoverDevice(I2Chandle_t *pI2C, uint8_t i)
{
uint8_t j, cTemp[8];
int iDevice = DEVICE_UNKNOWN;

  if (i == 0x28 || i == 0x29) // Probably a Bosch BNO055
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1); // CHIP_ID register
    if (cTemp[0] == 0xa0)
       return DEVICE_BNO055;
  }
  if (i == 0x3c || i == 0x3d) // Probably an OLED display
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1);
    cTemp[0] &= 0xbf; // mask off power on/off bit
    if (cTemp[0] == 0x8) // SH1106
       iDevice = DEVICE_SH1106;
    else if (cTemp[0] == 3 || cTemp[0] == 6)
       iDevice = DEVICE_SSD1306;
    return iDevice;
  }
  
  if (i == 0x34 || i == 0x35) // Probably an AXP202/AXP192 PMU chip
  {
    I2CReadRegister(pI2C, i, 0x03, cTemp, 1); // chip ID
    if (cTemp[0] == 0x41)
       return DEVICE_AXP202;
    else if (cTemp[0] == 0x03)
       return DEVICE_AXP192;
  }
 
  if (i == 0x38)  // Probably a FT6236G/FT6336G/FT6336U/FT6436 touch screen controller chip
  {               //  - likely 0x39 address valid as well but no test HW to verify
    I2CReadRegister(pI2C, i, 0xA0, cTemp, 1); // chip ID
    if (cTemp[0] == 0x00)
       return DEVICE_FT6236G;
    else if (cTemp[0] == 0x01)
       return DEVICE_FT6336G;
      else if (cTemp[0] == 0x02)
       return DEVICE_FT6336U;
      else if (cTemp[0] == 0x03)
       return DEVICE_FT6436;
  }
  
  if (i >= 0x40 && i <= 0x4f) // check for TI INA219 power measurement sensor
  {
    I2CReadRegister(pI2C, i, 0x00, cTemp, 2);
    if (cTemp[0] == 0x39 && cTemp[1] == 0x9f)
       return DEVICE_INA219;
  }
  
  // Check for Microchip 24AAXXXE64 family serial 2 Kbit EEPROM
  if (i >= 0x50 && i <= 0x57) {
    uint32_t u32Temp = 0;
    I2CReadRegister(pI2C, i, 0xf8, (uint8_t *)&u32Temp,
                    3); // check for Microchip's OUI
    if (u32Temp == 0x000004a3 || u32Temp == 0x00001ec0 ||
        u32Temp == 0x00d88039 || u32Temp == 0x005410ec)
      return DEVICE_24AAXXXE64;
  }

  if (i == 0x51)  // Probably a BM8563 RTC
  {               
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1); // Command/Status register 1
    // Serial.print("CSR1REG(0x00): ");
    // Serial.println(cTemp[0], HEX);
    if ((cTemp[0] & 0xDF) == 0x00)    // all bits clear in Normal, ignore BIT5(STOP)
    {
      I2CReadRegister(pI2C, i, 0x01, cTemp, 1); // Command/Status register 2
      // Serial.print("CSR2REG(0x01): ");
      // Serial.println(cTemp[0], HEX);
      if ((cTemp[0] & 0xE0) == 0x00)          // BIT5-7 always clear
      {
        I2CReadRegister(pI2C, i, 0x02, cTemp, 1); // seconds register 
        // Serial.print("SECREG(0x02):     ");
        // Serial.println(cTemp[0], HEX);   
        if ((cTemp[0] & 0x80) == 0x00)        // BIT7(VL) clear if PWRON    
          return DEVICE_BM8563;
      }
    }
  }


//  else if (i == 0x5b) // MLX90615?
//  {
//    I2CReadRegister(pI2C, i, 0x10, cTemp, 3);
//    for (j=0; j<3; j++) Serial.println(cTemp[j], HEX);
//  }
  // try to identify it from the known devices using register contents
  {    
    // Check for TI HDC1080
    I2CReadRegister(pI2C, i, 0xff, cTemp, 2);
    if (cTemp[0] == 0x10 && cTemp[1] == 0x50)
       return DEVICE_HDC1080;

    // Check for BME680
    if (i == 0x76 || i == 0x77)
    {
       I2CReadRegister(pI2C, i, 0xd0, cTemp, 1); // chip ID
       if (cTemp[0] == 0x61) // BME680
          return DEVICE_BME680;
    }
    // Check for VL53L0X
    I2CReadRegister(pI2C, i, 0xc0, cTemp, 3);
    if (cTemp[0] == 0xee && cTemp[1] == 0xaa && cTemp[2] == 0x10)
       return DEVICE_VL53L0X;

    // Check for CCS811
    I2CReadRegister(pI2C, i, 0x20, cTemp, 1);
    if (cTemp[0] == 0x81) // Device ID
       return DEVICE_CCS811;

    // Check for LIS3DSH accelerometer from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x3f) // WHO_AM_I
       return DEVICE_LIS3DSH;

    // Check for LIS3DH accelerometer from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x33) // WHO_AM_I
       return DEVICE_LIS3DH;

    // Check for LSM9DS1 magnetometer/gyro/accel sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0x68) // WHO_AM_I
       return DEVICE_LSM9DS1;

    // Check for LPS25H pressure sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0xbd) // WHO_AM_I
       return DEVICE_LPS25H;
    
    // Check for HTS221 temp/humidity sensor from STMicro
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
    if (cTemp[0] == 0xbc) // WHO_AM_I
       return DEVICE_HTS221;
    
    // Check for MAG3110
    I2CReadRegister(pI2C, i, 0x07, cTemp, 1);
    if (cTemp[0] == 0xc4) // WHO_AM_I
       return DEVICE_MAG3110;

    // Check for LM8330 keyboard controller
    I2CReadRegister(pI2C, i, 0x80, cTemp, 2);
    if (cTemp[0] == 0x0 && cTemp[1] == 0x84) // manufacturer code + software revision
       return DEVICE_LM8330;

    // Check for MAX44009
    if (i == 0x4a || i == 0x4b)
    {
      for (j=0; j<8; j++)
        I2CReadRegister(pI2C, i, j, &cTemp[j], 1); // check for power-up reset state of registers
      if ((cTemp[2] == 3 || cTemp[2] == 2) && cTemp[6] == 0 && cTemp[7] == 0xff)
         return DEVICE_MAX44009;
    }
       
    // Check for ADS1115
    I2CReadRegister(pI2C, i, 0x02, cTemp, 2); // Lo_thresh defaults to 0x8000
    I2CReadRegister(pI2C, i, 0x03, &cTemp[2], 2); // Hi_thresh defaults to 0x7fff
    if (cTemp[0] == 0x80 && cTemp[1] == 0x00 && cTemp[2] == 0x7f && cTemp[3] == 0xff)
       return DEVICE_ADS1115;

    // Check for MCP9808
    I2CReadRegister(pI2C, i, 0x06, cTemp, 2); // manufacturer ID && get device ID/revision
    I2CReadRegister(pI2C, i, 0x07, &cTemp[2], 2); // need to read them individually
    if (cTemp[0] == 0 && cTemp[1] == 0x54 && cTemp[2] == 0x04 && cTemp[3] == 0x00)
       return DEVICE_MCP9808;
       
    // Check for BMP280/BME280
    I2CReadRegister(pI2C, i, 0xd0, cTemp, 1);
    if (cTemp[0] == 0x55) // BMP180
       return DEVICE_BMP180;
    else if (cTemp[0] == 0x58)
       return DEVICE_BMP280;
    else if (cTemp[0] == 0x60) // BME280
       return DEVICE_BME280;

    // Check for LSM6DS3
    I2CReadRegister(pI2C, i, 0x0f, cTemp, 1); // WHO_AM_I
    if (cTemp[0] == 0x69)
       return DEVICE_LSM6DS3;
       
    // Check for ADXL345
    I2CReadRegister(pI2C, i, 0x00, cTemp, 1); // DEVID
    if (cTemp[0] == 0xe5)
       return DEVICE_ADXL345;

    // Check for MPU-6886
    if (i == 0x68 || i == 0x69)
    { 
      I2CReadRegister(pI2C, i, 0x75, cTemp, 1);   // WHO_AM_I
      if (cTemp[0] == 0x19)
        return DEVICE_MPU6886;
    }

    // Check for MPU-60x0i, MPU-688X (not MPU-6886) and MPU-9250
    I2CReadRegister(pI2C, i, 0x75, cTemp, 1);
    if (cTemp[0] == (i & 0xfe)) // Current I2C address (low bit set to 0)
       return DEVICE_MPU6000;
    else if (cTemp[0] == 0x71)
       return DEVICE_MPU9250;
    else if (cTemp[0] == 0x19)
        return DEVICE_MPU688X;


    // Check for DS3231 RTC
    I2CReadRegister(pI2C, i, 0x0e, cTemp, 1); // read the control register
    if (i == 0x68 &&
        cTemp[0] == 0x1c) // fixed I2C address and power on reset value
      return DEVICE_DS3231;

    // Check for DS1307 RTC
    I2CReadRegister(pI2C, i, 0x07, cTemp, 1); // read the control register
    if (i == 0x68 &&
        cTemp[0] == 0x03) // fixed I2C address and power on reset value
      return DEVICE_DS1307;
        
  }
  return iDevice;
} /* I2CDiscoverDevice() */
