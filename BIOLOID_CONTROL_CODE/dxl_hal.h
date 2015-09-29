/*
 * dxl_hal.h - Hardware abstraction layer functions for the serial port 
 * Dynamixel interface on the Robotis CM-510 controller. 
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.4		30/09/2011
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/

#ifndef _DYNAMIXEL_HAL_H
#define _DYNAMIXEL_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the USART0 with the specified baud rate
// devIndex is not used for initialization
int dxl_hal_open(int devIndex, float baudrate);

// closes the com port (not implemented)
void dxl_hal_close(void);

// clears the communication buffer
void dxl_hal_clear(void);

// send a packet of data of numPacket bytes
int dxl_hal_tx( unsigned char *pPacket, int numPacket );

// receive a packet of data of numPacket bytes
int dxl_hal_rx( unsigned char *pPacket, int numPacket );

// set the maximum waiting time for a given number of bytes to be received
void dxl_hal_set_timeout( int NumRcvByte );

// check for timeout during receive
int dxl_hal_timeout(void);

#ifdef __cplusplus
}
#endif

#endif
