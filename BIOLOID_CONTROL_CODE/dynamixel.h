/*
 * dynamixel.h - Functions for the Dynamixel interface on the 
 *   Robotis CM-510 controller. 
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.4		30/09/2011
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/

/*
 * Communication Protocol
 * 
 * Instruction Packet (TX from CM-510):
 * ------------------------------------
 * 2 Start Bytes:	0xFF 0xFF
 * Dynamixel ID:    id (single byte, OxFE = broadcast)
 * Packet Length:	number of parameters + 2
 * Instruction:		Instr (single byte, see AX-12 manual)
 * Parameters:		depend on instruction
 * Checksum:		~(ID+Length+Instruction+Parameters)
 *
 * Status Packet (RX from Dynamixel):
 * ------------------------------------
 * 2 Start Bytes:	0xFF 0xFF
 * Packet Length:	number of parameters + 2
 * Error Code:      Bit 0..6 encode the errors (see manual)
 * Parameters:		depends on instruction sent
 * Checksum:		~(ID+Length+Error+Parameters)
 *
 */

#ifndef _DYNAMIXEL_H
#define _DYNAMIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "global.h"

// default baud rate is 1MBps
#define DEFAULT_BAUDNUMBER	(1)

// Dynamixel device control table (see manual for details)
// EEPROM area 0-23 (0x00 - 0x17)
//		Item				  Address	Read/Write	Initial Value
#define DXL_MODEL_NUMBER_L		00	//	R			12
#define DXL_MODEL_NUMBER_H		01	//	R			0
#define DXL_FIRMWARE_VERSION	02	//	R			24 
#define DXL_ID					03	//	R/W			1
#define DXL_BAUD_RATE			04	//	R/W			1
#define DXL_RETURN_DELAY_TIME	05	//	R/W			250
#define DXL_CW_ANGLE_LIMIT_L	06	//	R/W			0
#define DXL_CW_ANGLE_LIMIT_H	07	//	R/W			0
#define DXL_CCW_ANGLE_LIMIT_L	08	//	R/W			255
#define DXL_CCW_ANGLE_LIMIT_H	09	//	R/W			3
#define DXL_TEMPERATURE_LIMIT	11	//	R/W			70 (C)*
#define DXL_LOW_VOLTAGE_LIMIT	12	//	R/W			70 (100mV)*
#define DXL_HIGH_VOLTAGE_LIMIT	13	//	R/W			140 (100mV)*
#define DXL_MAX_TORQUE_L		14	//	R/W			255
#define DXL_MAX_TORQUE_H		15	//	R/W			3
#define DXL_STATUS_RETURN_LEVEL	16	//	R/W			2
#define DXL_ALARM_LED			17	//	R/W			36 (overheat & overload)*
#define DXL_ALARM_SHUTDOWN		18	//	R/W			36 (overheat & overload)*
#define DXL_DOWN_CALIBRATION_L	20	//	R			?
#define DXL_DOWN_CALIBRATION_H	21	//	R			?
#define DXL_UP_CALIBRATION_L	22	//	R			?
#define DXL_UP_CALIBRATION_H	23	//	R			?
//* values are different from factory default settings
// RAM area 24-49 (0x18 - 0x31)
//		Item					 Address	Rd/Wr	Initial Value
#define DXL_TORQUE_ENABLE			24	//	R/W		0
#define DXL_LED						25	//	R/W		0
#define DXL_CW_COMPLIANCE_MARGIN	26	//	R/W		0
#define DXL_CCW_COMPLIANCE_MARGIN	27	//	R/W		0
#define DXL_CW_COMPLIANCE_SLOPE		28	//	R/W		32
#define DXL_CCW_COMPLIANCE_SLOPE	29	//	R/W		32
#define DXL_GOAL_POSITION_L			30	//	R/W		[ADDR36]value
#define DXL_GOAL_POSITION_H			31	//	R/W		[ADDR37]value
#define DXL_MOVING_SPEED_L			32	//	R/W		0
#define DXL_MOVING_SPEED_H			33	//	R/W		0
#define DXL_TORQUE_LIMIT_L			34	//	R/W		[ADDR14]value
#define DXL_TORQUE_LIMIT_H			35	//	R/W		[ADDR15]value
#define DXL_PRESENT_POSITION_L		36	//	R		?
#define DXL_PRESENT_POSITION_H		37	//	R		?
#define DXL_PRESENT_SPEED_L			38	//	R		?
#define DXL_PRESENT_SPEED_H			39	//	R		?
#define DXL_PRESENT_LOAD_L			40	//	R		?
#define DXL_PRESENT_LOAD_H			41	//	R		?
#define DXL_PRESENT_VOLTAGE			42	//	R		?
#define DXL_PRESENT_TEMPERATURE		43	//	R		?
#define DXL_REGISTERED_INSTRUCTION	44	//	R/W		0
#define DXL_MOVING					46	//	R		0
#define DXL_LOCK					47	//	R/W		0
#define DXL_PUNCH_L					48	//	R/W		32
#define DXL_PUNCH_H					49	//	R/W		0


// Use dxl_init as the top level initialization method 
void dxl_init(int baudnum);
// called by dxl_init 
int dxl_initialize( int devIndex, int baudnum );
void dxl_terminate();

// set/get packet methods
#define MAXNUM_TXPARAM		(150)
#define MAXNUM_RXPARAM		(60)

// set the Dynamixel Id for a the instruction packet
void dxl_set_txpacket_id(int id);
#define BROADCAST_ID		(254)

// set the instruction for a the instruction packet
void dxl_set_txpacket_instruction(int instruction);
#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)

// set a parameter for a the instruction packet
void dxl_set_txpacket_parameter(int index, int value);
// set the packet length for a the instruction packet
void dxl_set_txpacket_length(int length);

// unpack the error code received from the Dynamixel
int dxl_get_rxpacket_error(int errbit);
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

// get the status packet length
int dxl_get_rxpacket_length(void);
// get a parameter from the status packet
int dxl_get_rxpacket_parameter(int index);

// utility functions for byte/word operations
int dxl_makeword(int lowbyte, int highbyte);
int dxl_get_lowbyte(int word);
int dxl_get_highbyte(int word);

// packet communication methods 
void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_txrx_packet(void);

// retrieve the last error status
int dxl_get_result(void);
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL			(2)
#define COMM_RXFAIL			(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)

// high level communication methods 
// Ping a Dynamixel device
// returns the error bits from the status packet obtained
int dxl_ping(int id);

// Read data from the control table of a Dynamixel device
// Length 0x04, Instruction 0x02
// Parameter1 Starting address of the location where the data is to be read
// Parameter2 Length of the data to be read (one/two bytes in this case)
int dxl_read_byte(int id, int address);
int dxl_read_word(int id, int address);

// Function to write data into the control table of the Dynamixel actuator
// Length N+3 (N is the number of data to be written)
// Instruction 0x03
// Parameter1 Starting address of the location where the data is to be written
// Parameter2 1st data to be written
// Parameter3 2nd data to be written, etc.
// In this case we have 1 or 2 parameters respectively
// Returns communication status - see dxl_get_result
int dxl_write_byte(int id, int address, int value);
int dxl_write_word(int id, int address, int value);

// Supplementary functions to print communication errors (requires serial port to PC)
// Print error bit of status packet
void dxl_printErrorCode();
// Print communication result
void dxl_printCommStatus(int CommStatus);

// Function for controlling several Dynamixel actuators at the same time. 
// The communication time decreases by using the Sync Write instruction 
// since many instructions can be transmitted by a single instruction. 
// However, you can use this instruction only when the lengths and addresses 
// of the control table to be written to are the same. 
// The broadcast ID (0xFE) needs to be used for transmitting.
// ID: 0xFE
// Length: (L + 1) * N + 4 (L: Data length for each Dynamixel actuator, N: The number of Dynamixel actuators)
// Instruction: 0x83
// Parameter1 Starting address of the location where the data is to be written
// Parameter2 The length of the data to be written (L)
// Parameter3 The ID of the 1st Dynamixel actuator
// Parameter4 The 1st data for the 1st Dynamixel actuator
// Parameter5 The 2nd data for the 1st Dynamixel actuator
// ParameterL+4 The ID of the 2nd Dynamixel actuator
// ParameterL+5 The 1st data for the 2nd Dynamixel actuator
// ParameterL+6 The 2nd data for the 2nd Dynamixel actuator
// …
// NOTE: this function only allows 2 bytes of data per actuator
int dxl_sync_write_word( int NUM_ACTUATOR, int address, const uint8 ids[], int values[] );

// Function setting goal and speed for all Dynamixel actuators at the same time  
// Uses the Sync Write instruction (also see dxl_sync_write_word) 
// Inputs:	NUM_ACTUATOR - number of Dynamixel servos
//			ids - array of Dynamixel ids to write to
//			goal - array of goal positions
//			speed - array of moving speeds
//Returns:	commStatus
int dxl_set_goal_speed( int NUM_ACTUATOR, const uint8 ids[], uint16 goal[], uint16 speed[] );


#ifdef __cplusplus
}
#endif

#endif
