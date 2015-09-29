

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <ctype.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "global.h"
#include "serial.h"


// Command Strings List - kept in Flash to conserve RAM
const char COMMANDSTR0[]  PROGMEM = "STOP";
const char COMMANDSTR1[]  PROGMEM = "WFWD";
const char COMMANDSTR2[]  PROGMEM = "WBWD";
const char COMMANDSTR3[]  PROGMEM = "WLT ";
const char COMMANDSTR4[]  PROGMEM = "WRT ";
const char COMMANDSTR5[]  PROGMEM = "WLSD";
const char COMMANDSTR6[]  PROGMEM = "WRSD";
const char COMMANDSTR7[]  PROGMEM = "WFLS";
const char COMMANDSTR8[]  PROGMEM = "WFRS";
const char COMMANDSTR9[]  PROGMEM = "WBLS";
const char COMMANDSTR10[] PROGMEM = "WBRS";
const char COMMANDSTR11[] PROGMEM = "WAL ";
const char COMMANDSTR12[] PROGMEM = "WAR ";
const char COMMANDSTR13[] PROGMEM = "WFLT";
const char COMMANDSTR14[] PROGMEM = "WFRT";
const char COMMANDSTR15[] PROGMEM = "WBLT";
const char COMMANDSTR16[] PROGMEM = "WBRT";
const char COMMANDSTR17[] PROGMEM = "WRDY";
const char COMMANDSTR18[] PROGMEM = "SIT ";
const char COMMANDSTR19[] PROGMEM = "STND";
const char COMMANDSTR20[] PROGMEM = "BAL ";
const char COMMANDSTR21[] PROGMEM = "M   ";
const char COMMANDSTR22[] PROGMEM = "FGUP";
const char COMMANDSTR23[] PROGMEM = "BGUP";
const char COMMANDSTR24[] PROGMEM = "RSET";
PGM_P COMMANDSTR_POINTER[] PROGMEM = { 
COMMANDSTR0, COMMANDSTR1, COMMANDSTR2, COMMANDSTR3, COMMANDSTR4,
COMMANDSTR5, COMMANDSTR6, COMMANDSTR7, COMMANDSTR8, COMMANDSTR9,
COMMANDSTR10, COMMANDSTR11, COMMANDSTR12, COMMANDSTR13, COMMANDSTR14, 
COMMANDSTR15, COMMANDSTR16, COMMANDSTR17, COMMANDSTR18, COMMANDSTR19,
COMMANDSTR20, COMMANDSTR21, COMMANDSTR22, COMMANDSTR23, COMMANDSTR24 };

// set up the read buffer
volatile unsigned char gbSerialBuffer[MAXNUM_SERIALBUFF] = {0};
volatile unsigned char gbSerialBufferHead = 0;
volatile unsigned char gbSerialBufferTail = 0;
static FILE *device;

// global variables
extern volatile uint8 bioloid_command;			// current command
extern volatile uint8 last_bioloid_command;		// last command
extern volatile uint8 flag_receive_ready;		// received complete command flag
extern volatile uint8 current_motion_page;		// current motion page
extern volatile uint8 next_motion_page;			// next motion page if we got new command

// internal function prototypes
void serial_put_queue( unsigned char data );
unsigned char serial_get_queue(void);
int std_putchar(char c,  FILE* stream);
int std_getchar( FILE* stream );

// the new implementation of AVR libc does not allow variables passed to _delay_ms
static inline void delay_ms(uint16 count) {
	while(count--) { 
		_delay_ms(1); 
	} 
}


// ISR for serial receive, Serial Port/ZigBEE use USART1
SIGNAL(USART1_RX_vect)
{
	char c;
	
	c = UDR1;
	// check if we have received a CR+LF indicating complete string
	if (c == '\r')
	{
		// command complete, set flag and write termination byte to buffer
		flag_receive_ready = 1;
		serial_put_queue( 0xFF );
		c = '\n';
		std_putchar(c, device);
		// test
		std_putchar(' ', device);
	} 
	else
	{
		// put each received byte into the buffer until full
		serial_put_queue( c );
		// echo the character
		std_putchar(c, device);
	}
}

// initialize the serial port with the specified baud rate
void serial_init(long baudrate)
{
	unsigned short Divisor;

	// in case of ZigBee comms, enable the device
#ifdef ZIG_2_SERIAL
	DDRC  = 0x7F;
	PORTC = 0x7E;
	// to enable ZigBee communications we need PD5=low, PD6=high, make PD7 input and turn off pull-up on PD7
	PORTD &= ~0x80; 
	PORTD &= ~0x20; 
	PORTD |= 0x40; 
	// we need to wait for the connection to get established
	delay_ms(500);
#endif

	// set UART register A
	//Bit 7: USART Receive Complete
	//Bit 6: USART Transmit Complete
	//Bit 5: USART Data Register Empty 
	//Bit 4: Frame Error
	//Bit 3: Data OverRun
	//Bit 2: Parity Error
	//Bit 1: Double The USART Transmission Speed
	//Bit 0: Multi-Processor Communication Mode
	UCSR1A = 0b01000010;
	
	// set UART register B
	// bit7: enable rx interrupt
    // bit6: enable tx interrupt
    // bit4: enable rx
    // bit3: enable tx
    // bit2: set sending size(0 = 8bit)
	UCSR1B = 0b10011000;
	
	// set UART register C
	// bit6: communication mode (1 = synchronous, 0 = asynchronous)
    // bit5,bit4: parity bit(00 = no parity) 
    // bit3: stop bit(0 = stop bit 1, 1 = stop bit 2)
    // bit2,bit1: data size(11 = 8bit)
	UCSR1C = 0b00000110;

	// Set baud rate
	Divisor = (unsigned short)(2000000.0 / baudrate) - 1;
	UBRR0H = (unsigned char)((Divisor & 0xFF00) >> 8);
	UBRR0L = (unsigned char)(Divisor & 0x00FF);

	// initialize
	UDR1 = 0xFF;
	gbSerialBufferHead = 0;
	gbSerialBufferTail = 0;

	// open the serial device for printf()
	device = fdevopen( std_putchar, std_getchar );
	
	// reset commands and flags
	bioloid_command = 0;				
	last_bioloid_command = 0;		
	flag_receive_ready = 0;			
}

// Top level serial port task
// manages all requests to read from or write to the serial port
// Receives commands from the serial port and writes output (excluding printf)
// Checks the status flag provided by the ISR for operation
// Returns:  int flag = 0 when no new command has been received
//           int flag = 1 when new command has been received
int sendCommand(char *command)
{
	char c1, c2, c3, c4, buffer[6];
	int match;
    printf("Entro al sistema\n");
	// loop over all known commands to find a match
	for (uint8 i=0; i<NUMBER_OF_COMMANDS; i++)
	{
		// the command strings sit in PROGMEM - so need to use pgmspace.h macros
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(COMMANDSTR_POINTER[i])));
		match = strcmp(	buffer, command	);		
		if ( match == 0 )
		{		
			printf("Encontro coicidencia \n %s",command);	
			// we found a match set the command
			last_bioloid_command = bioloid_command;
			bioloid_command = i;
			break;
		} 
		
		// if we get to end of loop we haven't found a match
		if ( i== NUMBER_OF_COMMANDS-1 )
		{
			last_bioloid_command = bioloid_command;
			// 0xFF means no match found
			bioloid_command = COMMAND_NOT_FOUND;
		}
	}
	
		// cross-check against the definitions in global.h 
	   
		switch ( bioloid_command )
		{
			case COMMAND_WALK_READY:
				next_motion_page = COMMAND_WALK_READY_MP;
				break;
			case COMMAND_SIT:
				next_motion_page = COMMAND_SIT_MP;
				break;
			case COMMAND_STAND:
				next_motion_page = COMMAND_STAND_MP;
				break;
			case COMMAND_BALANCE:
				next_motion_page = COMMAND_BALANCE_MP;
				break;
			case COMMAND_BACK_GET_UP:
				next_motion_page = COMMAND_BACK_GET_UP_MP;
				break;
			case COMMAND_FRONT_GET_UP:
				next_motion_page = COMMAND_FRONT_GET_UP_MP;
				break;
			case COMMAND_RESET:
				next_motion_page = COMMAND_RESET_MP;
				break;
		}
	
	// otherwise it's easier to calculate the motion page for walk commands
	if( bioloid_command >= COMMAND_WALK_FORWARD  )
	{
		// all walk command motion pages are in sequence and 12 pages apart each
		printf("Valor comando %i\n",bioloid_command);
		next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
		printf("Valor pagina next motion %i\n",next_motion_page);
	}
	
	// before we leave we need to check for special case of Motion Page command
	if( bioloid_command == COMMAND_NOT_FOUND ) 
	{
		if ( c1 == 'M' && (c2 >= '0' && c2 <= '9') )
		{
			// we definitely have a motion page, find the number
			bioloid_command = COMMAND_MOTIONPAGE;
			next_motion_page = c2 - 48;	// converts ASCII to number
			// check if next character is still a number
			if ( c3 >= '0' && c3 <= '9' )
			{
				next_motion_page = next_motion_page * 10;
				next_motion_page += (c3-48); 
			}
			// check if next character is still a number
			if ( c4 >= '0' && c4 <= '9' )
			{
				next_motion_page = next_motion_page * 10;
				next_motion_page += (c4-48); 
			}
		}
	}
	
	// reset the flag
	flag_receive_ready = 0;
	
	// finally echo the command and write new command prompt
	if ( bioloid_command == COMMAND_MOTIONPAGE ) {
		printf( "%c%c%c%c - MotionPageCommand %i\n> ", c1, c2, c3, c4, next_motion_page );
	} else if( bioloid_command != COMMAND_NOT_FOUND ) {
		printf( "%c%c%c%c - Command # %i\n> ", c1, c2, c3, c4, bioloid_command );
	} else {
		printf( "%c%c%c%c \nUnknown Command! \n> ", c1, c2, c3, c4 );
	}	
	
	// set command received flag only if valid command
	if ( bioloid_command == COMMAND_NOT_FOUND ) {
		return 0;
	} else {
		return 1;
	}
}
int serialReceiveCommand()
{
	char c1, c2, c3, c4, command[6], buffer[6];
	int match;
	
	if (flag_receive_ready == 0)
	{
		// nothing to do, go straight back to main loop
		return 0;
	}
	
	// we have a new command, get characters 
	c1 = serial_get_queue();
	if ( c1 >= 'a' && c1 <= 'z' ) c1 = toupper(c1); // convert to upper case if required
	command[0] = c1;
	c2 = serial_get_queue();
	if ( c2 >= 'a' && c2 <= 'z' ) c2 = toupper(c2); // convert to upper case if required
	if( c2 == 0xFF ) c2 = ' ';						// pad with blanks to 4 characters
	command[1] = c2;				
	c3 = serial_get_queue();
	if ( c3 >= 'a' && c3 <= 'z' ) c3 = toupper(c3); // convert to upper case if required
	if( c3 == 0xFF ) c3 = ' ';						// pad with blanks to 4 characters
	command[2] = c3;
	c4 = serial_get_queue();
	if ( c4 >= 'a' && c4 <= 'z' ) c4 = toupper(c4); // convert to upper case if required
	if( c4 == 0xFF ) c4 = ' ';						// pad with blanks to 4 characters
	command[3] = c4;
	command[4] = 0x00;			// finish the string

	// flush the queue in case we received more than 4 bytes
	do 
	{
		// need to do it once even for 4 bytes to get rid of the 0xFF marking the end of string
		serial_get_queue();
	} while (serial_get_qstate() != 0);
	
	// loop over all known commands to find a match
	for (uint8 i=0; i<NUMBER_OF_COMMANDS; i++)
	{
		// the command strings sit in PROGMEM - so need to use pgmspace.h macros
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(COMMANDSTR_POINTER[i])));
		match = strcmp(	buffer, command	);		
		if ( match == 0 )
		{			
			// we found a match set the command
			last_bioloid_command = bioloid_command;
			bioloid_command = i;
			break;
		} 
		
		// if we get to end of loop we haven't found a match
		if ( i== NUMBER_OF_COMMANDS-1 )
		{
			last_bioloid_command = bioloid_command;
			// 0xFF means no match found
			bioloid_command = COMMAND_NOT_FOUND;
		}
	}
	
	// find the motion page associated with the command for non-walk commands
	if ( bioloid_command != COMMAND_NOT_FOUND && bioloid_command >= COMMAND_WALK_READY )
	{
		// cross-check against the definitions in global.h 
	   
		switch ( bioloid_command )
		{
			case COMMAND_WALK_READY:
				next_motion_page = COMMAND_WALK_READY_MP;
				break;
			case COMMAND_SIT:
				next_motion_page = COMMAND_SIT_MP;
				break;
			case COMMAND_STAND:
				next_motion_page = COMMAND_STAND_MP;
				break;
			case COMMAND_BALANCE:
				next_motion_page = COMMAND_BALANCE_MP;
				break;
			case COMMAND_BACK_GET_UP:
				next_motion_page = COMMAND_BACK_GET_UP_MP;
				break;
			case COMMAND_FRONT_GET_UP:
				next_motion_page = COMMAND_FRONT_GET_UP_MP;
				break;
			case COMMAND_RESET:
				next_motion_page = COMMAND_RESET_MP;
				break;
		}
	} 
	// otherwise it's easier to calculate the motion page for walk commands
	else if( bioloid_command >= COMMAND_WALK_FORWARD && bioloid_command < COMMAND_WALK_READY )
	{
		// all walk command motion pages are in sequence and 12 pages apart each
		printf("Valor comando %i\n",bioloid_command);
		next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
		printf("Valor pagina next motion %i\n",next_motion_page);
	}
	
	// before we leave we need to check for special case of Motion Page command
	if( bioloid_command == COMMAND_NOT_FOUND ) 
	{
		if ( c1 == 'M' && (c2 >= '0' && c2 <= '9') )
		{
			// we definitely have a motion page, find the number
			bioloid_command = COMMAND_MOTIONPAGE;
			next_motion_page = c2 - 48;	// converts ASCII to number
			// check if next character is still a number
			if ( c3 >= '0' && c3 <= '9' )
			{
				next_motion_page = next_motion_page * 10;
				next_motion_page += (c3-48); 
			}
			// check if next character is still a number
			if ( c4 >= '0' && c4 <= '9' )
			{
				next_motion_page = next_motion_page * 10;
				next_motion_page += (c4-48); 
			}
		}
	}
	
	// reset the flag
	flag_receive_ready = 0;
	
	// finally echo the command and write new command prompt
	if ( bioloid_command == COMMAND_MOTIONPAGE ) {
		printf( "%c%c%c%c - MotionPageCommand %i\n> ", c1, c2, c3, c4, next_motion_page );
	} else if( bioloid_command != COMMAND_NOT_FOUND ) {
		printf( "%c%c%c%c - Command # %i\n> ", c1, c2, c3, c4, bioloid_command );
	} else {
		printf( "%c%c%c%c \nUnknown Command! \n> ", c1, c2, c3, c4 );
	}	
	
	// set command received flag only if valid command
	if ( bioloid_command == COMMAND_NOT_FOUND ) {
		return 0;
	} else {
		return 1;
	}
}


// write out a data string to the serial port
void serial_write( unsigned char *pData, int numbyte )
{
	int count;

	for( count=0; count<numbyte; count++ )
	{
		// wait for the data register to empty
		while(!bit_is_set(UCSR1A,5));
		// before writing the next byte
		UDR1 = pData[count];
	}
}

// read a data string from the serial port
unsigned char serial_read( unsigned char *pData, int numbyte )
{
	int count, numgetbyte;
	
	// buffer is empty, nothing to read 
	if( gbSerialBufferHead == gbSerialBufferTail )
		return 0;
	
	// check number of bytes requested does not exceed whats in the buffer
	numgetbyte = serial_get_qstate();
	if( numgetbyte > numbyte )
		numgetbyte = numbyte;
	
	for( count=0; count<numgetbyte; count++ )
		pData[count] = serial_get_queue();
	
	// return how many bytes have been read
	return numgetbyte;
}

// get the number of bytes in the buffer
int serial_get_qstate(void)
{
	short NumByte;
	
	if( gbSerialBufferHead == gbSerialBufferTail )
		// buffer is empty
		NumByte = 0;
	// buffer is used in cyclic fashion
	else if( gbSerialBufferHead < gbSerialBufferTail )
		// head is to the left of the tail
		NumByte = gbSerialBufferTail - gbSerialBufferHead;
	else
		// head is to the right of the tail
		NumByte = MAXNUM_SERIALBUFF - (gbSerialBufferHead - gbSerialBufferTail);
	
	return (int)NumByte;
}

// puts a received byte into the buffer
void serial_put_queue( unsigned char data )
{
	// buffer is full, character is ignored
	if( serial_get_qstate() == (MAXNUM_SERIALBUFF-1) )
		return;
	
	// append the received byte to the buffer
	gbSerialBuffer[gbSerialBufferTail] = data;

	// have reached the end of the buffer, restart at beginning
	if( gbSerialBufferTail == (MAXNUM_SERIALBUFF-1) )
		gbSerialBufferTail = 0;
	else
	// move the tail by one byte
		gbSerialBufferTail++;
}

// get a byte out of the buffer
unsigned char serial_get_queue(void)
{
	unsigned char data;
	
	// buffer is empty, return 0xFF
	if( gbSerialBufferHead == gbSerialBufferTail )
		return 0xff;
	
	// buffer not empty, return next byte	
	data = gbSerialBuffer[gbSerialBufferHead];
		
	// head has reached end of buffer, restart at beginning
	if( gbSerialBufferHead == (MAXNUM_SERIALBUFF-1) )
		gbSerialBufferHead = 0;
	else
	// move the head by one byte
		gbSerialBufferHead++;
		
	return data;
}

// writes a single character to the serial port
// newline is replaced with CR+LF
int std_putchar(char c,  FILE* stream)
{
	char tx[2];
	
    if( c == '\n' )
	{
        tx[0] = '\r';
		tx[1] = '\n';
		serial_write( (unsigned char*)tx, 2 );
	}
	else
	{
		tx[0] = c;
		serial_write( (unsigned char*)tx, 1 );
	}
 
    return 0;
}

// get a single character out of the read buffer
// wait for byte to arrive if none in the buffer
int std_getchar( FILE* stream )
{
    char rx;
	
	while( serial_get_qstate() == 0 );
	rx = serial_get_queue();
	
	if( rx == '\r' )
		rx = '\n';

    return rx;
}
