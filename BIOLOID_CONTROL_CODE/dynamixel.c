/*
 * dynamixel.c - Functions for the Dynamixel interface on the 
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

#include <stdio.h>
#include <util/delay.h>
#include "global.h"
#include "dxl_hal.h"
#include "dynamixel.h"
#include "pose.h"

// define the positions of the bytes in the packet
#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)

// global hardware definition variables
extern const uint8 AX12_IDS[NUM_AX12_SERVOS];

// create the arrays that contain the instruction and status packet
unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10] = {0};
unsigned char gbStatusPacket[MAXNUM_RXPARAM+10] = {0};
// local shared variables 
unsigned char gbRxPacketLength = 0;
unsigned char gbRxGetLength = 0;
int gbCommStatus = COMM_RXSUCCESS;
int giBusUsing = 0;


// High level initialization - specific robot settings for Bioloid
void dxl_init(int baudnum)
{
	int commStatus = 0, errorStatus = 0;
	
	// now prepare the Dynamixel servos
	// first initialize the bus
	dxl_initialize( 0, baudnum ); 
	// wait 0.1s
	_delay_ms(100);
	
	// Next check the hardware configuration is valid
	for (int i=0; i<NUM_AX12_SERVOS; i++)
	{
		// ping each servo in turn
		errorStatus = dxl_ping(AX12_IDS[i]);
		if (errorStatus == -1)
		{
			printf("\nHardware Configuration Failure at Dynamixel ID %i.\n", AX12_IDS[i]);
			dxl_terminate();
			return;
		}
	}
	
	// set alarm LED and shutdown to prevent overheat/overload
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_ALARM_LED, 36);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_ALARM_LED Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_ALARM_SHUTDOWN, 36);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_ALARM_LED Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	// now set temperature and voltage limits
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_TEMPERATURE_LIMIT, 70);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_TEMPERATURE_LIMIT Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_LOW_VOLTAGE_LIMIT, 70);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_LOW_VOLTAGE_LIMIT Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	// set a 2-point compliance margin (equals 0.58 deg)
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_CW_COMPLIANCE_MARGIN, 2);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_CW_COMPLIANCE_MARGIN Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_CCW_COMPLIANCE_MARGIN, 2);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_CCW_COMPLIANCE_MARGIN Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	_delay_ms(100);
	// and enable torque to keep positions
	commStatus = dxl_write_byte(BROADCAST_ID, DXL_TORQUE_ENABLE, 1);
	if(commStatus != COMM_RXSUCCESS) {
		printf("\nDXL_TORQUE_ENABLE Broadcast - ");
		dxl_printCommStatus(dxl_get_result());
	}	
	_delay_ms(50);
}

// Initialize communication
int dxl_initialize( int devIndex, int baudnum )
{
	// set baud rate based on the baud number table 
	float baudrate;	
	baudrate = 2000000.0f / (float)(baudnum + 1);
	
	// open serial communication
	if( dxl_hal_open(devIndex, baudrate) == 0 )
		return 0;

	// show success and bus as free
	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
	return 1;
}

// Terminate communication
void dxl_terminate()
{
	dxl_hal_close();
}

// Send an instruction packet
void dxl_tx_packet()
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

	// do nothing if bus is busy
	if( giBusUsing == 1 )
		return;
	
	// set bus as busy
	giBusUsing = 1;

	// check packet does not exceed maximum number of parameters
	if( gbInstructionPacket[LENGTH] > (MAXNUM_TXPARAM+2) )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	// check instruction is valid
	if( gbInstructionPacket[INSTRUCTION] != INST_PING
		&& gbInstructionPacket[INSTRUCTION] != INST_READ
		&& gbInstructionPacket[INSTRUCTION] != INST_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_REG_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_ACTION
		&& gbInstructionPacket[INSTRUCTION] != INST_RESET
		&& gbInstructionPacket[INSTRUCTION] != INST_SYNC_WRITE )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	// create the packet header (2x 0xFF)
	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	// calculate the checksum
	for( i=0; i<(gbInstructionPacket[LENGTH]+1); i++ )
		checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
	// if timeout or corrupt clear the buffer
	if( gbCommStatus == COMM_RXTIMEOUT || gbCommStatus == COMM_RXCORRUPT )
		dxl_hal_clear();

	// transfer the packet
	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	RealTxNumByte = dxl_hal_tx( (unsigned char*)gbInstructionPacket, TxNumByte );

	// check that all bytes were sent 
	if( TxNumByte != RealTxNumByte )
	{
		// communication failure - not all bytes sent
		gbCommStatus = COMM_TXFAIL;
		giBusUsing = 0;
		return;
	}

	// for read instructions we expect a reply within the timeout period
	if( gbInstructionPacket[INSTRUCTION] == INST_READ )
		dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] + 6 );
	else
		dxl_hal_set_timeout( 6 );

	gbCommStatus = COMM_TXSUCCESS;
}

// receive a status packet
void dxl_rx_packet()
{
	unsigned char i, j, nRead;
	unsigned char checksum = 0;

	// return if bus is busy
	if( giBusUsing == 0 )
		return;

	// if instruction was broadcast there is nothing to wait for
	if( gbInstructionPacket[ID] == BROADCAST_ID )
	{
		gbCommStatus = COMM_RXSUCCESS;
		giBusUsing = 0;
		return;
	}
	
	// check transmission was successful and reset 
	if( gbCommStatus == COMM_TXSUCCESS )
	{
		gbRxGetLength = 0;
		gbRxPacketLength = 6;
	}
	
	// start reading what has been received on the bus
	nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
	gbRxGetLength += nRead;
	// not finished yet, check for timeout
	if( gbRxGetLength < gbRxPacketLength )
	{
		if( dxl_hal_timeout() == 1 )
		{
			// timeout, return an error
			if(gbRxGetLength == 0)
				gbCommStatus = COMM_RXTIMEOUT;
			else
				gbCommStatus = COMM_RXCORRUPT;
			giBusUsing = 0;
			return;
		}
	}
	
	// Find packet header
	for( i=0; i<(gbRxGetLength-1); i++ )
	{
		if( gbStatusPacket[i] == 0xff && gbStatusPacket[i+1] == 0xff )
		{
			break;
		}
		else if( i == gbRxGetLength-2 && gbStatusPacket[gbRxGetLength-1] == 0xff )
		{
			break;
		}
	}	
	// remove the header and copy to beginning of array
	if( i > 0 )
	{
		for( j=0; j<(gbRxGetLength-i); j++ )
			gbStatusPacket[j] = gbStatusPacket[j + i];
			
		gbRxGetLength -= i;		
	}

	// if we haven't received all bytes yet we are still waiting
	if( gbRxGetLength < gbRxPacketLength )
	{
		gbCommStatus = COMM_RXWAITING;
		return;
	}

	// Check id pairing, we only want response from the right Dynamixel
	if( gbInstructionPacket[ID] != gbStatusPacket[ID])
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
	// check the length of the status packet to see if we expect more
	gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
	if( gbRxGetLength < gbRxPacketLength )
	{
		// more to come - keep reading
		nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
		gbRxGetLength += nRead;
		if( gbRxGetLength < gbRxPacketLength )
		{
			gbCommStatus = COMM_RXWAITING;
			return;
		}
	}

	// Check checksum
	for( i=0; i<(gbStatusPacket[LENGTH]+1); i++ )
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;

	if( gbStatusPacket[gbStatusPacket[LENGTH]+3] != checksum )
	{
		// checksum does not match
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
	// everything is fine, return success
	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
}

// send instruction packet ans wait for reply
void dxl_txrx_packet()
{
	// send instruction packet
	dxl_tx_packet();

	// send was not successful, return error
	if( gbCommStatus != COMM_TXSUCCESS )
		return;	
	
	// wait for reply within the timeout period
	do{
		dxl_rx_packet();		
	}while( gbCommStatus == COMM_RXWAITING );	
}

// retrieve the last error status
int dxl_get_result()
{
	return gbCommStatus;
}

// set the Dynamixel Id for a the instruction packet
void dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

// set the instruction for a the instruction packet
void dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

// set a parameter for a the instruction packet
void dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

// set the packet length for a the instruction packet
void dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

// unpack the error code received from the Dynamixel
int dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}

// get the status packet length
int dxl_get_rxpacket_length()
{
	return (int)gbStatusPacket[LENGTH];
}

// get a parameter from the status packet
int dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

// combine 2 bytes into a word
int dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

// extract lower byte from a word
int dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

// extract higher byte from a word
int dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

// Ping a Dynamixel device
int dxl_ping( int id )
{
	// wait for the bus to be free
	while(giBusUsing);

	// create a PING instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_txrx_packet();
	
	if (gbCommStatus == COMM_RXSUCCESS)
	{
		// return the error code
		return (int)gbStatusPacket[ERRBIT];
	// check if servo exists (via timeout)
	} else if( gbCommStatus == COMM_RXTIMEOUT )
	{
		return -1;
	} else {
		return 0;
	}
}

// Read data from the control table of a Dynamixel device
// Length 0x04, Instruction 0x02
// Parameter1 Starting address of the location where the data is to be read
// Parameter2 Length of the data to be read (one byte in this case)
int dxl_read_byte( int id, int address )
{
	// wait for the bus to be free
	while(giBusUsing);

	// create a READ instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}

// Function to write data into the control table of the Dynamixel actuator
// Length N+3 (N is the number of data to be written)
// Instruction 0x03
// Parameter1 Starting address of the location where the data is to be written
// Parameter2 1st data to be written
// Parameter3 2nd data to be written, etc.
// In this case we only have a 1-byte parameter
int dxl_write_byte( int id, int address, int value )
{
	// wait for the bus to be free
	while(giBusUsing);

	// create a WRITE instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();
	
	return gbCommStatus;
}

// Read data from the control table of a Dynamixel device
// Length 0x04, Instruction 0x02
// Parameter1 Starting address of the location where the data is to be read
// Parameter2 Length of the data to be read (2 bytes in this case)
int dxl_read_word( int id, int address )
{
	// wait for the bus to be free
	while(giBusUsing);

	// create a READ instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	// combine the 2 bytes into a word and return
	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}

// Function to write data into the control table of the Dynamixel actuator
// Length N+3 (N is the number of data to be written)
// Instruction 0x03
// Parameter1 Starting address of the location where the data is to be written
// Parameter2 1st data to be written
// Parameter3 2nd data to be written, etc.
// In this case we have a two 1-byte parameters
int dxl_write_word( int id, int address, int value )
{
	// wait for the bus to be free
	while(giBusUsing);

	// create a WRITE instruction packet and send
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_txrx_packet();
	
	return gbCommStatus;
}

// Print communication result
void dxl_printCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmitting instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed to get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Waiting to receive status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: Status packet not received!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("Unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void dxl_printErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}

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
int dxl_sync_write_word( int NUM_ACTUATOR, int address, const uint8 ids[], int16 values[] )
{
	int i = 0;

	// wait for the bus to be free
	while(giBusUsing);

	// check how many actuators are to be broadcast to
	if (NUM_ACTUATOR == 0) {
		// nothing to do, return
		return 0;
	} else if (NUM_ACTUATOR == 1) {
		// easy, we can use dxl_write_word for a single actuator
		dxl_write_word( ids[0], address, values[0] );
		return 0;
	}
	
	// Multiple values, create sync write packet
	// ID is broadcast id
	dxl_set_txpacket_id(BROADCAST_ID);
	// Instruction is sync write
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	// Starting address where to write to
	dxl_set_txpacket_parameter(0, address);
	// Length of data to be written (each word = 2 bytes)
	dxl_set_txpacket_parameter(1, 2);
	// Loop over the active Dynamixel id's  
	for( i=0; i<NUM_ACTUATOR; i++ )
	{
		// retrieve the id and value for each actuator and add to packet
		dxl_set_txpacket_parameter(2+3*i, ids[i]);
		dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(values[i]));
		dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(values[i]));
	}
	
	// total length is as per formula above with L=2
	dxl_set_txpacket_length((2+1)*NUM_ACTUATOR + 4);
	
	// all done, send the packet
	dxl_txrx_packet();
	
	// there is no status packet return, so return the CommStatus
	return gbCommStatus;
}


// Function setting goal and speed for all Dynamixel actuators at the same time  
// Uses the Sync Write instruction (also see dxl_sync_write_word) 
// Inputs:	NUM_ACTUATOR - number of Dynamixel servos
//			ids - array of Dynamixel ids to write to
//			goal - array of goal positions
//			speed - array of moving speeds
//Returns:	commStatus
int dxl_set_goal_speed( int NUM_ACTUATOR, const uint8 ids[], uint16 goal[], uint16 speed[] )
{
	int i = 0;

	// wait for the bus to be free
	while(giBusUsing);

	// check how many actuators are to be broadcast to
	if (NUM_ACTUATOR == 0) {
		// nothing to do, return
		return 0;
	} 
	
	// Multiple values, create sync write packet
	// ID is broadcast id
	dxl_set_txpacket_id(BROADCAST_ID);
	// Instruction is sync write
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	// Starting address where to write to
	dxl_set_txpacket_parameter(0, DXL_GOAL_POSITION_L);
	// Length of data to be written (2 words = 4 bytes)
	dxl_set_txpacket_parameter(1, 4);
	// Loop over the active Dynamixel id's  
	for( i=0; i<NUM_ACTUATOR; i++ )
	{
		// retrieve the id and value for each actuator and add to packet
		dxl_set_txpacket_parameter(2+5*i, ids[i]);
		dxl_set_txpacket_parameter(2+5*i+1, dxl_get_lowbyte(goal[i]));
		dxl_set_txpacket_parameter(2+5*i+2, dxl_get_highbyte(goal[i]));
		dxl_set_txpacket_parameter(2+5*i+3, dxl_get_lowbyte(speed[i]));
		dxl_set_txpacket_parameter(2+5*i+4, dxl_get_highbyte(speed[i]));
	}
	
	// total length is as per formula above with L=4
	dxl_set_txpacket_length((4+1)*NUM_ACTUATOR + 4);
	
	// all done, send the packet
	dxl_txrx_packet();
	
	// there is no status packet return, so return the CommStatus
	return gbCommStatus;
}


