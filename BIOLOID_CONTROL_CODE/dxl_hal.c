/*
 * dxl_hal.c - Hardware abstraction layer functions for the serial port 
 * Dynamixel interface on the Robotis CM-510 controller. 
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.4		30/09/2011
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "dxl_hal.h"

// maximum buffer length is 256 bytes
#define MAXNUM_DXLBUFF	256
// Set the direction of communication and buffering
#define DIR_TXD 	PORTE &= ~0x08, PORTE |= 0x04
#define DIR_RXD 	PORTE &= ~0x04, PORTE |= 0x08

// create the buffer 
volatile unsigned char gbDxlBuffer[MAXNUM_DXLBUFF] = {0};
volatile unsigned char gbDxlBufferHead = 0;
volatile unsigned char gbDxlBufferTail = 0;
// timing variables for determining communication timeout
volatile double gfByteTransTime_us;
volatile unsigned int gwCountNum;
volatile unsigned int gwTimeoutCountNum;
volatile unsigned int gwReturnDelayCountNum;

// function prototypes for internal functions
int dxl_hal_get_qstate(void);
void dxl_hal_put_queue( unsigned char data );
unsigned char dxl_hal_get_queue(void);


// ISR for serial receive, Dynamixel Bus uses USART0
SIGNAL(USART0_RX_vect)
{
	dxl_hal_put_queue( UDR0 );
}

// Initialize the serial Dynamixel bus on USART0
int dxl_hal_open(int devIndex, float baudrate)
{
	// Opening device
	// devIndex: Device index (not used)
	// baudrate: Real baudrate (ex> 115200, 57600, 38400...)
	// Return: 0(Failed), 1(Succeed)
	
	unsigned short Divisor;

	// set UART register A
	//Bit 7: USART Receive Complete
	//Bit 6: USART Transmit Complete
	//Bit 5: USART Data Register Empty 
	//Bit 4: Frame Error
	//Bit 3: Data OverRun
	//Bit 2: Parity Error
	//Bit 1: Double USART Transmission Speed (asynchronous)
	//Bit 0: Multi-Processor Communication Mode
	UCSR0A = 0b01000010;
	
	// set UART register B
	// bit7: enable RX interrupt
    // bit6: enable TX interrupt
    // bit4: enable RX
    // bit3: enable TX
    // bit2: set sending size(0 = 8bit)
	UCSR0B = 0b10011000;
	
	// set UART register C
	// bit6: communication mode (1 = synchronous, 0 = asynchronous)
    // bit5,bit4: parity bit(00 = no parity) 
    // bit3: stop bit(0 = stop bit 1, 1 = stop bit 2)
    // bit2,bit1: data size(11 = 8bit)
	UCSR0C = 0b00000110;
	
	// Set baudrate
	Divisor = (unsigned short)(2000000.0 / baudrate) - 1;
	UBRR0H = (unsigned char)((Divisor & 0xFF00) >> 8);
	UBRR0L = (unsigned char)(Divisor & 0x00FF);

	gfByteTransTime_us = 1000000.0 / (double)baudrate * 12.0;
	gwReturnDelayCountNum = (unsigned int)(250.0 / gfByteTransTime_us);
	
	// initialize
	DIR_RXD;
	UDR0 = 0xFF;
	gbDxlBufferHead = 0;
	gbDxlBufferTail = 0;
	return 1;
}

// the new implementation of AVR libc does not allow variables passed to _delay_us
static inline void delay_us(double delay_time) {
	uint32_t count = (uint32_t) delay_time;
	while(count--) { 
		_delay_us(1); 
	} 
}

// close communication on Dynamixel bus
void dxl_hal_close(void)
{
	// Close serial communication on USART0
    // bit4: 0 - disable RX
    // bit3: 0 - disable TX
	UCSR0B = 0b00000000;
}

void dxl_hal_clear(void)
{
	// Clear communication buffer
	gbDxlBufferHead = gbDxlBufferTail;
}

// Function to transmit packet of data
// *pPacket: data array pointer
// numPacket: number of data array
// Return: number of data transmitted. -1 is error.	
int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	int count;
	
	// disable interrupts
	cli();
	// set direction to transmit
	DIR_TXD;
	
	// loop over packet of data and send
	for( count=0; count<numPacket; count++ )
	{
		// wait until data register is empty
		while(!bit_is_set(UCSR0A,5));
		// clear transmit complete flag
		UCSR0A |= 0x40;
		UDR0 = pPacket[count];
	}
	// wait for transmission to complete
	while( !bit_is_set(UCSR0A,6) );
	// set direction back to receive
	DIR_RXD;
	// re-enable interrupts
	sei();
	
	return count;
}

// Function to receive packet of data
// *pPacket: data array pointer
// numPacket: number of data array
// Return: number of data received. -1 is error.
int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	int count, numgetbyte;
	
	// buffer is empty, nothing to receive
	if( gbDxlBufferHead == gbDxlBufferTail )
		return 0;
	
	// check number of bytes requested does not exceed whats in the buffer
	numgetbyte = dxl_hal_get_qstate();
	if( numgetbyte > numPacket )
		numgetbyte = numPacket;
	
	// create data packet from the buffer
	for( count=0; count<numgetbyte; count++ )
		pPacket[count] = dxl_hal_get_queue();
	
	// return how many bytes have been received
	return numgetbyte;
}

// Start stop watch
// NumRcvByte: number of receiving data(to calculate maximum waiting time)
void dxl_hal_set_timeout( int NumRcvByte )
{
	gwCountNum = 0;	
	gwTimeoutCountNum = (NumRcvByte + 10) + gwReturnDelayCountNum;
}

// Check timeout
// Return: 0 is false, 1 is true(timeout occurred)
int dxl_hal_timeout(void)
{
	gwCountNum++;
		
	if( gwCountNum > gwTimeoutCountNum )
	{
		return 1;
	}
	
	delay_us(gfByteTransTime_us);
	return 0;
}

// get the number of bytes in the buffer
int dxl_hal_get_qstate(void)
{
	short NumByte;
	
	if( gbDxlBufferHead == gbDxlBufferTail )
		// buffer is empty
		NumByte = 0;
	// buffer is used in cyclic fashion
	else if( gbDxlBufferHead < gbDxlBufferTail )
		// head is to the left of the tail
		NumByte = gbDxlBufferTail - gbDxlBufferHead;
	else
		// head is to the right of the tail
		NumByte = MAXNUM_DXLBUFF - (gbDxlBufferHead - gbDxlBufferTail);
	
	return (int)NumByte;
}

// puts a received byte into the buffer
void dxl_hal_put_queue( unsigned char data )
{
	// buffer is full, character is ignored
	if( dxl_hal_get_qstate() == (MAXNUM_DXLBUFF-1) )
		return;
		
	// append the received byte to the buffer
	gbDxlBuffer[gbDxlBufferTail] = data;

	// have reached the end of the buffer, restart at beginning
	if( gbDxlBufferTail == (MAXNUM_DXLBUFF-1) )
		gbDxlBufferTail = 0;
	else
	// move the tail by one byte
		gbDxlBufferTail++;
}

// get a byte out of the buffer
unsigned char dxl_hal_get_queue(void)
{
	unsigned char data;
	
	// buffer is empty, return 0xFF
	if( gbDxlBufferHead == gbDxlBufferTail )
		return 0xff;
		
	// buffer not empty, return next byte	
	data = gbDxlBuffer[gbDxlBufferHead];
		
	// have reached the end of the buffer, restart at beginning
	if( gbDxlBufferHead == (MAXNUM_DXLBUFF-1) )
		gbDxlBufferHead = 0;
	else
	// move the head by one byte
		gbDxlBufferHead++;
		
	return data;
}



