/*****************************************************************************
*
*
* File              twi.cpp
* Date              Saturday, 10/29/17
* Composed by 		lucullus
*
* Modified by Benoit3 on 18/5/2019 to add multibyte send
* Modified by ChrisHul @ github.com on 11/03/2022 (major slave op update)
*  Enhanced SCL stretching by resetting overflow flag on exit only
*    also obviates conflict loading overflow counter and releasing
*    SCL at the same time.
*  Non-locking Start-vector ISR
*  #define instead of enum for slave send states. New states added
*  Postponing Onrequest invokation until sending of first byte
*  Corrected Twi_slave_send() bug
*
*  **** See twi.h for Credits and Usage information ****
*
*
*  This library is free software; you can redistribute it and/or modify it under the
*  terms of the GNU General Public License as published by the Free Software
*  Foundation; either version 2.1 of the License, or any later version.
*  This program is distributed in the hope that it will be useful, but WITHOUT ANY
*  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
*  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*
******************************************************************************/

#ifndef __USI_TWI_CPP__
#define __USI_TWI_CPP__


/*--------------------------------------------------------------
 includes
----------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi.h"


/*--------------------------------------------------------------
 local variables
----------------------------------------------------------------*/
// stores state of twi slave communication
// changed from enum variable that used uint16 variable
#define  USI_SLAVE_START_ADDRESS_READING          0x00
#define  USI_SLAVE_CHECK_ADDRESS                  0x01
#define  USI_SLAVE_LOAD_SENDING_DATA              0x02
#define  USI_SLAVE_SEND_DATA                      0x03
#define  USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA   0x04
#define  USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA     0x05
#define  USI_SLAVE_REQUEST_DATA                   0x06
#define  USI_SLAVE_GET_DATA_AND_SEND_ACK          0x07
#define  USI_SLAVE_RESET                          0x08
#define  USI_SLAVE_INVALID_STATE		  0x09

// stores twi master configuration or it's error state
union  USI_TWI_state
{
  unsigned char errorState;         // Can reuse the TWI_state for error states since it will not be needed if there is an error.
  struct
  {
    unsigned char addressMode         : 1;
    unsigned char masterWriteDataMode : 1;
    unsigned char memReadMode	      : 1;
    unsigned char unused              : 5;
  }; 
}   USI_TWI_state;

struct USI_TWI_MASTER_TRANSFER_RESULT
{
  unsigned char result;
  unsigned char error_code;
};

static uint8_t                  slaveAddress;
static volatile uint8_t         overflowState;
static volatile bool twi_bus_busy = false;
static volatile bool twi_master_mode = false;
static volatile bool currently_receiving = false;

// Receive ringbuffer with extra bytecounter, to give out the number of bytes without iterating through it
static uint8_t          rxBuf[ TWI_RX_BUFFER_SIZE ];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static volatile uint8_t rxByteNum;

// Transmit ringbuffer
static uint8_t          txBuf[ TWI_TX_BUFFER_SIZE ];
static volatile uint8_t txHead;
static volatile uint8_t txTail;

// Event function variables: called in case of a slave request or a slave receive
static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(int);

/*--------------------------------------------------------------
 local functions
----------------------------------------------------------------*/

// flushes the TWI buffers
static void flushTwiBuffers(void)
{
  rxTail = 0;
  rxHead = 0;
  txTail = 0;
  txHead = 0;
  rxByteNum = 0;
}

static inline void SET_USI_TO_SEND_ACK( bool ack)
{
  /* load counter and clear all interrupt flags except overflow */
  USISR =
       ( 1 << USI_START_COND_INT ) | ( 0 << USIOIF ) |
       ( 1 << USIPF ) | ( 1 << USIDC ) |
       /* set USI counter to shift 1 bit */
       ( 0x0E << USICNT0 );

  /* prepare ACK */
  if ( ack) USIDR = 0; else USIDR = 0xFF;
  /* set SDA as output -- last thing to do in order to reduce noise */
  DDR_USI |= ( 1 << PORT_USI_SDA );
}

static inline void SET_USI_TO_READ_ACK( )
{
  /* set SDA as input */
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  /* load counter and clear all interrupt flags except overflow */
  USISR =
       ( 1 << USI_START_COND_INT ) | ( 0 << USIOIF ) |
       ( 1 << USIPF ) | ( 1 << USIDC ) |
       /* set USI counter to shift 1 bit */
       ( 0x0E << USICNT0 );
  /* prepare ACK */
  USIDR = 0;
}

/* setup TWI so that it will detect a Start Condition */
/* when it occurs it will trigger an interrupt */
static inline void SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( )
{
  twi_bus_busy = false;
  // set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA);

  USISR =
        /* clear all interrupt flags */
        ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) |
        ( 1 << USIDC ) | ( 0x0 << USICNT0 );
  USICR =
       /* enable Start Condition Interrupt, disable Overflow Interrupt */
       ( 1 << USISIE ) | ( 0 << USIOIE ) |
       /* set USI in Two-wire mode, no USI Counter overflow SCL stretch */
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       /* Shift Register Clock Source = External, positive edge */
       /* 4-Bit Counter Source = external, both edges */
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       /* no toggle clock-port pin */
       ( 0 << USITC );
  overflowState = USI_SLAVE_INVALID_STATE;

}

static inline void SET_USI_TO_SEND_DATA( uint8_t data)
{
  /* clear all interrupt flags except overflow */
  USISR    = 
       ( 1 << USI_START_COND_INT ) | ( 0 << USIOIF ) | ( 1 << USIPF ) |
       ( 1 << USIDC) |
       /* set USI to shift out 8 bits */
       ( 0x0 << USICNT0 );

  USIDR = data; // set output shift register
  /* set SDA as output -- last thing to do in order to reduce noise */
  DDR_USI |=  ( 1 << PORT_USI_SDA );
}

static inline void SET_USI_TO_READ_DATA( )
{
  /* set SDA as input */
  DDR_USI &= ~( 1 << PORT_USI_SDA );
  /* clear all interrupt flags - leave USIOIF until last*/
  USISR    =
       ( 1 << USI_START_COND_INT ) | ( 0 << USIOIF ) |
       ( 1 << USIPF ) | ( 1 << USIDC ) |
       /* set USI to shift in 8 bits */
       ( 0x0 << USICNT0 );
}

/*---------------------------------------------------------------
 Core function of Master for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
USI_TWI_MASTER_TRANSFER_RESULT USI_TWI_Master_Transfer( unsigned char temp, bool ack)
{
  USI_TWI_MASTER_TRANSFER_RESULT result;
  result.error_code = 0;
  USISR = temp;                                     // Set USISR according to temp.
                                                    // Prepare clocking.
  temp  =  (0<<USISIE)|(0<<USIOIE)|                 // Interrupts disabled
           (1<<USIWM1)|(0<<USIWM0)|                 // Set USI in Two-wire mode.
           (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|     // Software clock strobe as source.
           (1<<USITC);                              // Toggle Clock Port.
  do
  { 
	_delay_us(T2_TWI);
    USICR = temp;                          // Generate positve SCL edge.
    while( !(PIN_USI & (1<<PIN_USI_SCL)) );// Wait for SCL to go high.
	_delay_us(T4_TWI);
    USICR = temp;                          // Generate negative SCL edge.
    #ifdef BUS_ARBITRATION
      if(!ack && (USISR & (1<<USIDC))) { // Check data collision bit
        Twi_slave_init(slaveAddress);
        result.result = 0;
        result.error_code = USI_TWI_ARBITRATION_LOST;
        return result;
      }
    #endif
  }while( !(USISR & (1<<USIOIF)) );        // Check for transfer complete.
  
	_delay_us(T2_TWI);
  temp  = USIDR;                           // Read out data.
  USIDR = 0xFF;                            // Release SDA.
  DDR_USI |= (1<<PIN_USI_SDA);             // Enable SDA as output.

  result.result = temp;

  return result;                             // Return the data from the USIDR
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition. 
---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Start( void )
{
/* Release SCL to ensure that (repeated) Start can be performed */
  PORT_USI |= (1<<PIN_USI_SCL);                     // Release SCL.
  while( !(PORT_USI & (1<<PIN_USI_SCL)) );          // Verify that SCL becomes high.
  _delay_us(T2_TWI);

/* Generate Start Condition */
  PORT_USI &= ~(1<<PIN_USI_SDA);                    // Force SDA LOW.
	_delay_us(T4_TWI);                         
  PORT_USI &= ~(1<<PIN_USI_SCL);                    // Pull SCL LOW.
  PORT_USI |= (1<<PIN_USI_SDA);                     // Release SDA.

#ifdef SIGNAL_VERIFY
  if( !(USISR & (1<<USISIF)) )
  {
    USI_TWI_state.errorState = USI_TWI_MISSING_START_CON;  
    return (FALSE);
  }
#endif
  return (TRUE);
}

/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Stop( void )
{
  PORT_USI &= ~(1<<PIN_USI_SDA);           // Pull SDA low.
  PORT_USI |= (1<<PIN_USI_SCL);            // Release SCL.
  while( !(PIN_USI & (1<<PIN_USI_SCL)) );  // Wait for SCL to go high.  
	_delay_us(T4_TWI);
  PORT_USI |= (1<<PIN_USI_SDA);            // Release SDA.
	_delay_us(T2_TWI);
  
#ifdef SIGNAL_VERIFY
  if( !(USISR & (1<<USIPF)) )
  {
    USI_TWI_state.errorState = USI_TWI_MISSING_STOP_CON;    
    return (FALSE);
  }
#endif

  return (TRUE);
}

/*---------------------------------------------------------------
 Master Twi function. First it sends a Start-Condition. The first
 byte in parameter msg is always the
 slave address with the r/w-bit. Corresponding to the r/w-bit the
 function sends all bytes with index < msgSize, or transmits only
 the slave address and then reads msgSize-1 bytes from the slave.
 Finally it sends a Stop-Condition
---------------------------------------------------------------*/
bool USI_TWI_Start_Read_Write( unsigned char *msg, unsigned char msgSize)
{
	unsigned char const tempUSISR_8bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
                                 (0x0<<USICNT0);                                     // set USI to shift 8 bits i.e. count 16 clock edges.
	unsigned char const tempUSISR_1bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
                                 (0xE<<USICNT0); 									// set USI to shift 1 bit i.e. count 2 clock edges.
	unsigned char *savedMsg;
	unsigned char savedMsgSize;

	USI_TWI_state.errorState = 0; // Clear error and mode bits 

  USI_TWI_state.addressMode = TRUE;			// Always true for first byte

#ifdef PARAM_VERIFICATION
  if(msg > (unsigned char*)RAMEND)                 // Test if address is outside SRAM space
  {
    USI_TWI_state.errorState = USI_TWI_DATA_OUT_OF_BOUND;
    return (FALSE);
  }
  if(msgSize <= 1)                                 // Test if the transmission buffer is empty
  {
    USI_TWI_state.errorState = USI_TWI_NO_DATA;
    return (FALSE);
  }
#endif

#ifdef NOISE_TESTING                                // Test if any unexpected conditions have arrived prior to this execution.
  /*if( USISR & (1<<USISIF) ) // seems to always occur. Even, if nothing bad happened. Currently disabled
  {
    USI_TWI_state.errorState = USI_TWI_UE_START_CON;
    return (FALSE);
  }*/
  if( USISR & (1<<USIPF) )
  {
    USI_TWI_state.errorState = USI_TWI_UE_STOP_CON;
    return (FALSE);
  }
  if( USISR & (1<<USIDC) )
  {
    USI_TWI_state.errorState = USI_TWI_UE_DATA_COL;
    return (FALSE);
  }
#endif

  if ( !(*msg & (1<<TWI_READ_BIT)) )                // The LSB in the address byte determines if is a masterRead or masterWrite operation.
  {
    USI_TWI_state.masterWriteDataMode = TRUE;
  }

//	if (USI_TWI_state.memReadMode)
//	{
		savedMsg = msg;
		savedMsgSize = msgSize;
//	}

	if ( !USI_TWI_Master_Start( ))
  {
	return (FALSE);                           // Send a START condition on the TWI bus.
  }

/*Write address and Read/Write data */
  do
  {
    /* If masterWrite cycle (or inital address tranmission)*/
    if (USI_TWI_state.addressMode || USI_TWI_state.masterWriteDataMode)
    {
      /* Write a byte */
      PORT_USI &= ~(1<<PIN_USI_SCL);                // Pull SCL LOW.
      USIDR     = *(msg++);                        // Setup data.
      unsigned char error_code = USI_TWI_Master_Transfer( tempUSISR_8bit, false ).error_code; // Send 8 bits on bus.
      if(error_code!=0){ // Check if arbitration is lost
        USI_TWI_state.errorState = error_code;
        return (FALSE);
      }
      /* Clock and verify (N)ACK from slave */
      DDR_USI  &= ~(1<<PIN_USI_SDA);                // Enable SDA as input.
      USI_TWI_MASTER_TRANSFER_RESULT temp_result = USI_TWI_Master_Transfer( tempUSISR_1bit, true );
      // No Check for Arbitration Lost, since the arbitration cannot be lost on (N)ACK
      if( temp_result.result & (1<<TWI_NACK_BIT) ) 
      {
        if ( USI_TWI_state.addressMode )
          USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_ADDRESS;
        else
          USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_DATA;
        return (FALSE);
      }
	  
  	  if ((!USI_TWI_state.addressMode) && USI_TWI_state.memReadMode)// means memory start address has been written
  	  {
    		msg = savedMsg;					// start at slave address again
    		*(msg) |= (TRUE<<TWI_READ_BIT);  // set the Read Bit on Slave address
    		USI_TWI_state.errorState = 0;
    		USI_TWI_state.addressMode = TRUE;	// Now set up for the Read cycle
    		msgSize = savedMsgSize;				// Set byte count correctly
    		// NOte that the length should be Slave adrs byte + # bytes to read + 1 (gets decremented below)
    		if ( !USI_TWI_Master_Start( ))
    		{
    			USI_TWI_state.errorState = USI_TWI_BAD_MEM_READ;
    			return (FALSE);                           // Send a START condition on the TWI bus.
    		}
  	  }
  	  else
  	  {
  		  USI_TWI_state.addressMode = FALSE;            // Only perform address transmission once.
  	  }
    }
    /* Else masterRead cycle*/
    else
    {
      /* Read a data byte */
      DDR_USI   &= ~(1<<PIN_USI_SDA);               // Enable SDA as input.
      USI_TWI_MASTER_TRANSFER_RESULT temp_result = USI_TWI_Master_Transfer( tempUSISR_8bit,false );
      if( temp_result.error_code != 0){ // Check for Arbitration Lost
        USI_TWI_state.errorState = temp_result.error_code;
        return (FALSE);
      }
      *(msg++)  = temp_result.result;

      /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
      if( msgSize == 1)                            // If transmission of last byte was performed.
      {
        USIDR = 0xFF;                              // Load NACK to confirm End Of Transmission.
      }
      else
      {
        USIDR = 0x00;                              // Load ACK. Set data register bit 7 (output for SDA) low.
      }
      USI_TWI_Master_Transfer( tempUSISR_1bit,true ); // Generate ACK/NACK.
      // No Check for Arbitration Lost, since the arbitration cannot be lost on (N)ACK
    }
  }while( --msgSize) ;                             // Until all data sent/received.
  
  if (!USI_TWI_Master_Stop())
  {
	return (FALSE);                           // Send a STOP condition on the TWI bus.
	}

/* Transmission successfully completed*/
  return (TRUE);
}



/*--------------------------------------------------------------
 public functions
----------------------------------------------------------------*/

/*
 Disable USI module
*/
void Twi_end(){
	USICR &= 0b11001111; // disable USI
	DDR_USI &= ~(( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA )); // set SDA & SCL as input
  twi_master_mode = false;
  twi_bus_busy = false;
}

/* 
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called after a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void Twi_attachSlaveRxEvent( void (*function)(int) )
{
  twi_onSlaveReceive = function;
}

/* 
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void Twi_attachSlaveTxEvent( void (*function)(void) )
{
  twi_onSlaveTransmit = function;
}

void Twi_slave_init(uint8_t slave_addr)
{
  flushTwiBuffers( );
  twi_master_mode = false;

  slaveAddress = slave_addr;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will stretch SCL
  // when a start condition is detected followed by SCL low or a counter
  // overflow (latter only for USIWM1, USIWM0 = 11).
  // USIWM0 will be reset in SET_USI_TO_TWI_WAIT_FOR_START_CONDITION
  // and set to 1 in START_CONDITION ISR

  // set SCL and SDA high. Should remain so in slave mode
  PORT_USI |= ( 1 << PORT_USI_SCL | 1 << PORT_USI_SDA);
  DDR_USI |= 1 << PORT_USI_SCL; // SCL as output ( will enable SCL stretch)

  SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( );

  PIN_CHANGE_MASK |= 1 << PIN_USI_SDA; // enable pin change interrupt on PCINT0 (SDA line)
  GENERAL_INTERRUPT_FLAGS  |= 1 << PIN_CHANGE_FLAG; // clear interrupt flag, by writing 1 to it
  GENERAL_INTERRUPT_MASK |= 1 << PIN_CHANGE_INTERRUPT_ENABLE; // enable Pin Change Interrupt
}

uint8_t Twi_slave_send(uint8_t data)
{
  uint8_t tmphead;

  // calculate buffer index
  tmphead = ( txHead + 1 ) & TWI_TX_BUFFER_MASK;

  // wait for free space in buffer
  if(tmphead == txTail) return 0;

  // store data in buffer
  txBuf[ txHead ] = data;

  // store new index
  txHead = tmphead;
  return 1;
}

uint8_t Twi_slave_send(uint8_t *data, uint8_t length)
{
  uint8_t count=0;
  
  //send byte one by one as long as there's free space in the buffer
  while ( (count < length) && (Twi_slave_send(data[count])==1)) count++;

  //return count of transmitted bytes
  return count;
}

uint8_t Twi_receive(void)
{
  // wait for Rx data
  while ( rxHead == rxTail );

  // calculate buffer index
  rxTail = ( rxTail + 1 ) & TWI_RX_BUFFER_MASK;
  rxByteNum--;

  // return data from the buffer.
  return rxBuf[ rxTail ];
}

/*
 Returns the number of bytes in rx buffer
*/
uint8_t Twi_available(void)
{
  return rxByteNum;
}

//Use this function to get hold of the error message from the last transmission
unsigned char USI_TWI_Get_State_Info( void )
{
  return ( USI_TWI_state.errorState );                            // Return error state.
}


void Twi_master_init(void)
{
  twi_master_mode = true;

  GIMSK &= ~(1 << PIN_CHANGE_INTERRUPT_ENABLE);
  slaveAddress = 0;

  USIDR    =  0xFF;                       // Preload dataregister with "released level" data.
  USICR    =  (0<<USISIE)|(0<<USIOIE)|                            // Disable Interrupts.
              (1<<USIWM1)|(0<<USIWM0)|                            // Set USI in Two-wire mode.
              (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|                // Software stobe as counter clock source
              (0<<USITC);

  USISR   =   (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Clear flags,
              (0x0<<USICNT0);                                     // and reset counter.

  PORT_USI |= (1<<PIN_USI_SDA);           // Enable pullup on SDA, to set high as released state.
  PORT_USI |= (1<<PORT_USI_SCL);           // Enable pullup on SCL, to set high as released state.

  DDR_USI |= ( 1 << PORT_USI_SDA );
  DDR_USI |= ( 1 << PORT_USI_SCL );
}

void Twi_master_beginTransmission(uint8_t slave_addr)
{
	flushTwiBuffers( );

	// calculate buffer index
	txHead = ( txHead + 1 ) & TWI_TX_BUFFER_MASK;

	// store slave address in buffer
	txBuf[ txHead ] = (slave_addr<<TWI_ADR_BITS) | USI_SEND;
}

uint8_t Twi_master_send(uint8_t data)
{
	uint8_t temphead = (txHead + 1) & TWI_TX_BUFFER_MASK;
	if( temphead == txTail ) return 0; // return 0 if buffer is full

	txBuf[txHead] = data;

	txHead = temphead;

  return 1;
}

uint8_t Twi_master_send(uint8_t *data, uint8_t length)
{
  uint8_t count=0;
  
  //send byte one by one as long as there's free space in the buffer
  while ( (count < length) && (Twi_master_send(data[count])==1)) count++;

  //return count of transmitted bytes
  return count;
}

uint8_t Twi_master_endTransmission()
{
	uint8_t tempbuf[TWI_TX_BUFFER_SIZE];
	uint8_t j=0;
  bool ok=false;
  // copy tx buffer to tempory buffer
	while( txHead != txTail ){
		txTail = ( txTail +1 ) & TWI_TX_BUFFER_MASK;
		tempbuf[j] = txBuf[ txTail ];
		j++;
	}

  if(twi_bus_busy) {USI_TWI_state.errorState = USI_TWI_BUS_BUSY; return USI_TWI_state.errorState;}
	
	if(USI_TWI_Start_Read_Write(tempbuf,j)) return 0;
	else return USI_TWI_Get_State_Info();
}

uint8_t Twi_master_requestFrom(uint8_t slave_addr, uint8_t numBytes)
{
	uint8_t tempbuf[TWI_RX_BUFFER_SIZE];
	bool transferOK= false;
	numBytes++; // add extra byte to transmit header
	tempbuf[0] = (slave_addr<<TWI_ADR_BITS) | USI_RCVE;   // setup address & Rcve bit
  if(twi_bus_busy) {USI_TWI_state.errorState = USI_TWI_BUS_BUSY; return USI_TWI_state.errorState;}
	transferOK = USI_TWI_Start_Read_Write(tempbuf,numBytes);

	for(uint8_t i=0;i < TWI_RX_BUFFER_SIZE;i++){ // swap data from tempbuffer to rxBuffer
		uint8_t temphead = (rxHead + 1) & TWI_RX_BUFFER_MASK;
		
		rxBuf[temphead] = tempbuf[i];
		rxByteNum++;

		rxHead = temphead;
	}
	if(transferOK) return 0;
	else return USI_TWI_Get_State_Info();
}


/*--------------------------------------------------------------
 Interrupt Routines,  implement slave functions
----------------------------------------------------------------*/

// Interrupt Routine, triggered when master sends start condition
ISR( USI_START_VECTOR )
{

  twi_bus_busy = true;

  if ( currently_receiving) {
      // if we had received something, this is a RESTART condition 
      // and we have to call the users receive callback
      twi_onSlaveReceive(rxByteNum);
      currently_receiving = false;
  }


// setup to detect SCL going low before setting up address listening

  USISR =
       // clear interrupt flags
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
       ( 1 << USIPF ) |( 1 << USIDC ) |
       ( 0x0F << USICNT0); /* detect one SCL transition (going low) */
  USICR =
         // keep Start Condition Interrupt enabled to detect RESTART
         ( 1 << USISIE ) |
         // enable Overflow Interrupt as well
         ( 1 << USIOIE ) |
         // set USI in Two-wire mode, stretch SCL on USI Counter overflow
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
         // Shift Register Clock Source = External, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

    overflowState = USI_SLAVE_START_ADDRESS_READING;

  if ( !(PIN_USI & ( 1 << PIN_USI_SCL )))
  {
    // SCL already low - setup for address detection instead
      USISR =
         // clear interrupt flags
         ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
         ( 1 << USIPF ) | ( 1 << USIDC ) |
         // set USI to sample 8 bits (count 16 external SCL pin toggles)
         ( 0x0 << USICNT0); // shift in 8 bits

      // set default starting conditions for new TWI package
      // - listen for slave address
      overflowState = USI_SLAVE_CHECK_ADDRESS;
  }

  USISR |= 1 << USI_START_COND_INT; // release SCL if stretched

} // end ISR( USI_START_VECTOR )


// Interrupt Routine, triggered whenever CLK transition counter overflows
ISR( USI_OVERFLOW_VECTOR )
{
  switch ( overflowState )
  {
    // SCL gone low after START condition
    case USI_SLAVE_START_ADDRESS_READING:

      overflowState = USI_SLAVE_CHECK_ADDRESS;
      SET_USI_TO_READ_DATA( );

      break;

    // Address mode: check address and send ACK if match continue with
    // receive/transmit if OK, else reset USI
    case USI_SLAVE_CHECK_ADDRESS:
      if ( ( USIDR == 0 ) || ( ( USIDR >> 1 ) == slaveAddress) )
      {
        if ( (USIDR & 0x01 )) // master read
        {
          // read request: check if function is available
          // but don't request data from user until address
          // acknowledged. This is done to minimize lag between
          // address reception and acknowledge
          if( twi_onSlaveTransmit)
	  {
            // setup so that response data from user loads
            // after ack is sent
            overflowState = USI_SLAVE_LOAD_SENDING_DATA;
	    SET_USI_TO_SEND_ACK( true);
	  }
	  else
	  {
            // no onrequest available. Deny request
            overflowState = USI_SLAVE_RESET;
	    SET_USI_TO_SEND_ACK( false);
	  }
        }
        else
        {
          currently_receiving = true;
          overflowState = USI_SLAVE_REQUEST_DATA;
          SET_USI_TO_SEND_ACK( true);
        } // end if master read
      }
      else
      {
        SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( );
      }
      break;

    case USI_SLAVE_LOAD_SENDING_DATA:

      txHead = txTail = 0;
      twi_onSlaveTransmit(); // user onrequest callback

      goto _USI_SLAVE_SEND_DATA; // go on sending the first byte


    // Master write data mode: check reply and goto USI_SLAVE_SEND_DATA if OK,
    // else reset USI
    case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
      if ( USIDR )
      {
        // if NACK, the master does not want more data
        SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( );
        break;
      }
      // from here we just drop straight into USI_SLAVE_SEND_DATA if the
      // master sent an ACK

    // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
    case USI_SLAVE_SEND_DATA:

_USI_SLAVE_SEND_DATA:

      // Get data from Buffer
      if ( txHead == txTail )
      {
        // the buffer is empty -- release bus
        // not possible to signal error to master
//        SET_USI_TO_READ_ACK( ); // really needed?
        SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( );
      }
      else
      {
        overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
        SET_USI_TO_SEND_DATA( txBuf[ txTail ]);
        txTail = ( txTail + 1 ) & TWI_TX_BUFFER_MASK;
      }
      break;

    // set USI to sample reply from master
    // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
    case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
      overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
      SET_USI_TO_READ_ACK( );
      break;

    // Master read data mode: set USI to sample data from master, next
    // USI_SLAVE_GET_DATA_AND_SEND_ACK
    case USI_SLAVE_REQUEST_DATA:
      overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA( );
      break;

    // copy data from USIDR and send ACK
    // next USI_SLAVE_REQUEST_DATA
    case USI_SLAVE_GET_DATA_AND_SEND_ACK:
      // put data into buffer
      // Not necessary, but prevents warnings
      rxHead = ( rxHead + 1 ) & TWI_RX_BUFFER_MASK;
      rxBuf[ rxHead ] = USIDR;
      rxByteNum++;
      // next USI_SLAVE_REQUEST_DATA
      overflowState = USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK( true);
      break;

    // Only used if onrequest has not been set ( master read)
    // after NACK
    case USI_SLAVE_RESET:
    case USI_SLAVE_INVALID_STATE:
      SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( );
      break;

  } // end switch
  _delay_us(1);
  USISR |= 1 << USIOIF; // release CLK
} // end ISR( USI_OVERFLOW_VECTOR )


// Interrupt Routine, triggered when SDA pin changes; for detecting Stop condition
// Check Stop Condition flag and Start interrupt enable

ISR( PCINT0_vect )
{
//  if( ( USISR & ( 1 << USIPF)) && !( DDR_USI & ( 1 << PORT_USI_SDA )) &&
  if( ( USISR & ( 1 << USIPF)) && 
      ( PIN_USI & ( 1 << PIN_USI_SCL )) && ( PIN_USI & ( 1 << PIN_USI_SDA )))
  {

        if(currently_receiving) { // If we are receiving bytes from a master, call user callback
          if(rxByteNum>0) twi_onSlaveReceive(rxByteNum);  // check, if anything is in the rx buffer, because it is possible,
                                                        // that the communication was interrupted by a stop condition
          currently_receiving = false;
        }
        SET_USI_TO_TWI_WAIT_FOR_START_CONDITION( );

   }
   USISR |= 1 << USIPF;
}

#endif
