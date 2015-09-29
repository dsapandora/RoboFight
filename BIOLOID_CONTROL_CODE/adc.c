/*
 * ADC.c - Library for using the analog inputs on the Robotis CM-510
 *  controller. Reads ADC0 for battery voltage and ADC1-ADC6 from 
 *	the external 5-pin ports. By default ports are assigned as follows:
 *		GyroX = CM-510 Port3 = ADC3 = PORTF3
 *		GyroY = CM-510 Port4 = ADC4 = PORTF4
 *		DMS   = CM-510 Port5 = ADC5 = PORTF5
 *
 * Version 0.5 - Created: 31/10/2011 
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * Based on the Pololu library (see below)
 */ 

/*
 * Written by Ben Schmidel, May 27, 2008.
 * Copyright (c) 2008 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *   http://www.pololu.com/docs/0J18
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>
#include "global.h"
#include "adc.h"
#include "clock.h"
#include "buzzer.h"

// Global variables related to the finite state machine that governs execution
extern volatile uint8 bioloid_command;			// current command
extern volatile uint8 last_bioloid_command;		// last command
// and also the current and next motion pages
extern volatile uint8 current_motion_page;
extern volatile uint8 next_motion_page;	
extern uint8 current_step;						// number of the current motion page step
// joint offset values
extern volatile int16 joint_offset[NUM_AX12_SERVOS];

// global variables used for the ADC values
extern volatile uint8 adc_sensor_enable[ADC_CHANNELS];  // enabled sensors
extern volatile int16 adc_sensor_val[ADC_CHANNELS]; 	 // array of sensor values
extern volatile uint16 adc_battery_val;		 // battery voltage in millivolts
extern volatile uint16 adc_gyrox_center;	 // gyro x center value
extern volatile uint16 adc_gyroy_center;	 // gyro x center value
extern volatile int16 adc_accelx;			 // accelerometer x value
extern volatile int16 adc_accely;			 // accelerometer y value
extern volatile uint16 adc_accelx_center;	 // accelerometer x center value
extern volatile uint16 adc_accely_center;	 // accelerometer y center value
extern volatile uint16 adc_ultrasonic_distance;	// ultrasonic distance sensor value
extern volatile uint16 adc_dms_distance;   // DMS sensor distance value

uint16 millivolt_calibration = 5000;	// contains default VCC in millivolts
int16 fwd_bwd_balance = 0;				// gyro x deviation from center
int16 left_right_balance = 0;			// gyro y deviation from center

// buzzer alarm melody
extern const char melody5[];

#define DMSTablePoints	11	// number of entries in DMS conversion table 
// Tables to convert DMS values to distance in cm
// Values are from actual measurements, not the diagram in the e-manual
const uint16 DMSTableValues[DMSTablePoints]=
{70, 80, 95, 115, 150, 175, 205, 245, 310, 455, 625};
const uint16 DMSTableCM[DMSTablePoints]=
{80, 70, 60, 50, 40, 30, 25, 20, 15, 10, 5};

// internal timing related variables that control when the sensors are read
unsigned long last_gyro_read = 0;
unsigned long last_dms_read = 0;
unsigned long last_battery_read = 0;


// function to process the sensor data when new data become available
// detects slips (robot has fallen over forward/backward)
// and also low battery alarms at this stage
// Returns:  int flag = 0 no new command
//           int flag = 1 new command
int adc_processSensorData()
{
	int16 fb_joint_offset1, fb_joint_offset2, rl_joint_offset0, rl_joint_offset1;

	// check battery voltage still within limits
	if ( adc_battery_val < LOW_VOLTAGE_CUTOFF ) {
		// too low - play alarm and stop 
		buzzer_playFromProgramSpace(melody5);
		last_bioloid_command = bioloid_command;
		bioloid_command = COMMAND_SIT;
		next_motion_page = COMMAND_SIT_MP;
		return 1;
	}
	
	// calculate gyro deviations from center values
	fwd_bwd_balance = adc_sensor_val[ADC_GYROX-1] - (int16) adc_gyrox_center;
	left_right_balance = adc_sensor_val[ADC_GYROY-1] - (int16) adc_gyroy_center;
	// calculate accelerations as per ADXL203 datasheet
	adc_accelx = adc_sensor_val[ADC_ACCELX-1] - adc_accelx_center;	// center value is ~2500mV, produces acceleration in mg
	adc_accely = adc_sensor_val[ADC_ACCELY-1] - adc_accely_center;	// center value is ~2500mV, produces acceleration in mg
	// calculate distance from Maxbotix EZ0 sensor (avoid floating point calculations for speed reasons)
	adc_ultrasonic_distance = adc_sensor_val[ADC_ULTRASONIC-1] >> 2;	// gives approximate distance in cm (true factor is 0.259cm per mV)
	adc_dms_distance = adc_convertDMStoCM(adc_sensor_val[ADC_DMS-1]);	// gives approximate distance in cm (no interpolation)
	
	// TEST: printf("\n%i, %i, %i, %i, %i, %i, %i", current_motion_page, current_step, fwd_bwd_balance, left_right_balance, adc_accelx, adc_accely, adc_ultras);
	
	// did read sensors - check if robot slipped
	if( fwd_bwd_balance > GYROX_SLIP_ERROR ) {
		// backward slip
		last_bioloid_command = bioloid_command;
		bioloid_command = COMMAND_BACK_GET_UP;
		next_motion_page = COMMAND_BACK_GET_UP_MP;
		return 1;
	}
	else if( fwd_bwd_balance < -GYROX_SLIP_ERROR ) {
		// forward slip
		last_bioloid_command = bioloid_command;
		bioloid_command = COMMAND_FRONT_GET_UP;
		next_motion_page = COMMAND_FRONT_GET_UP_MP;
		return 1;
	}
	
	// calculate joint offset values as per Robotis Task files
	fb_joint_offset1 = (fwd_bwd_balance<<2) / 54;		// knee servo adjustment
	fb_joint_offset2 = fb_joint_offset1 * 3;			// ankle servo uses 3x as much offset
	rl_joint_offset1 = (left_right_balance<<2) / 40;	// hip servo adjustment
	rl_joint_offset0 = rl_joint_offset1 * 2;			// ankle servo uses 2x as much offset
	
	// TEST: printf("\nOffsets FB = %i, %i, RL= %i, %i", fb_joint_offset1, fb_joint_offset2, rl_joint_offset0, rl_joint_offset1);
	
	// just in case reset all offset values
	for (uint8 i=0; i<NUM_AX12_SERVOS; i++) {
		joint_offset[i] = 0;
	}
	// and apply to servos - this code is dependent on the hardware configuration
#ifdef HUMANOID_TYPEA	// Type A - all 18 servos are present and numbers match
	joint_offset[13-1] = fb_joint_offset1;
	joint_offset[15-1] = fb_joint_offset2;
	joint_offset[14-1] = -fb_joint_offset1;
	joint_offset[16-1] = -fb_joint_offset2;
	joint_offset[9-1]  = rl_joint_offset1;
	joint_offset[10-1] = rl_joint_offset1;
	joint_offset[17-1] = -rl_joint_offset0;
	joint_offset[18-1] = -rl_joint_offset0;
#endif
#ifdef HUMANOID_TYPEB	// Type B - 16 servos and 9 and 10 are missing
	joint_offset[13-3] = fb_joint_offset1;
	joint_offset[15-3] = fb_joint_offset2;
	joint_offset[14-3] = -fb_joint_offset1;
	joint_offset[16-3] = -fb_joint_offset2;
	joint_offset[17-3] = -rl_joint_offset0;
	joint_offset[18-3] = -rl_joint_offset0;
#endif
#ifdef HUMANOID_TYPEC	// Type C - 16 servos and 7 and 8 are missing
	joint_offset[13-3] = fb_joint_offset1;
	joint_offset[15-3] = fb_joint_offset2;
	joint_offset[14-3] = -fb_joint_offset1;
	joint_offset[16-3] = -fb_joint_offset2;
	joint_offset[9-3]  = rl_joint_offset1;
	joint_offset[10-3] = rl_joint_offset1;
	joint_offset[17-3] = -rl_joint_offset0;
	joint_offset[18-3] = -rl_joint_offset0;
#endif

	return 0;
}

// function that reads all the sensors from the main loop
// SENSOR_READ_INTERVAL in global.h determines how often the sensors are read
// BATTERY_READ_INTERVAL in global.h determines how often the battery voltage is read
// Returns:  int flag = 0 when no new values have been read
//           int flag = 1 when new values have been read
int adc_readSensors()    
{
	// reading the battery has no impact on return value, so done first
	if( (millis() - last_battery_read) >= BATTERY_READ_INTERVAL ) 	
	{
		adc_battery_val = adc_readBatteryMillivolts();
		// reset the timing variable
		last_battery_read = millis();
	}

	// check if we are overdue for reading the sensors
	if( (millis() - last_gyro_read) >= GYRO_READ_INTERVAL ) 
	{
		// read each sensor in sequence
		// single conversion time is around 120us
		if (adc_sensor_enable[ADC_GYROX-1] == 1)
		{
			adc_sensor_val[ADC_GYROX-1] = adc_read(ADC_GYROX);
		}
		if (adc_sensor_enable[ADC_GYROY-1] == 1)
		{
			adc_sensor_val[ADC_GYROY-1] = adc_read(ADC_GYROY);
		}
		if (adc_sensor_enable[ADC_ACCELX-1] == 1)
		{
			adc_sensor_val[ADC_ACCELX-1] = adc_readMillivolts(ADC_ACCELX);	
		}
		if (adc_sensor_enable[ADC_ACCELY-1] == 1)
		{
			adc_sensor_val[ADC_ACCELY-1] = adc_readMillivolts(ADC_ACCELY);
		}
		// only read distance sensors if they are due
		if( (millis() - last_dms_read) >= DMS_READ_INTERVAL )
		{
			if (adc_sensor_enable[ADC_DMS-1] == 1)
			{
				adc_sensor_val[ADC_DMS-1] = adc_read(ADC_DMS);
			}
			if (adc_sensor_enable[ADC_ULTRASONIC-1] == 1)
			{
				adc_sensor_val[ADC_ULTRASONIC-1] = adc_readMillivolts(ADC_ULTRASONIC);  
			}
			last_dms_read = millis();
		}		
		// reset the timing variable
		last_gyro_read = millis();
		return 1;
	}
	
	// didn't update the sensors
	return 0;
}

// Initialization for the ADC and sensor readings
void adc_init()
{
	uint16 mv_calibration = 0;
	
	// read the internal reference voltage
	mv_calibration = adc_readVCCMillivolts();
	// set the millivolt calibration accordingly
	adc_setMillivoltCalibration(mv_calibration);

	// now check the battery voltage
	adc_battery_val = adc_readBatteryMillivolts();
	
	// and set the timing variables
	last_battery_read = millis();

	// finally we need to find initial gyro and accelerometer center positions
	adc_gyrox_center = 0;
	adc_gyroy_center = 0;
	adc_accelx_center = 0;
	adc_accely_center = 0;
	// we take 12 samples every 32ms for 500ms
	// also check each sensor is enabled
	for (uint8 i=0; i<16; i++)
	{
		if (adc_sensor_enable[ADC_GYROX-1] == 1)
		{
			adc_gyrox_center += adc_readAverage(ADC_GYROX,12);
		}
		if (adc_sensor_enable[ADC_GYROY-1] == 1)
		{
			adc_gyroy_center += adc_readAverage(ADC_GYROY,12);
		}
		if (adc_sensor_enable[ADC_ACCELX-1] == 1)
		{
			adc_accelx_center += adc_readAverageMillivolts(ADC_ACCELX,12);
		}
		if (adc_sensor_enable[ADC_ACCELY-1] == 1)
		{
			adc_accely_center += adc_readAverageMillivolts(ADC_ACCELY,12);
		}
		_delay_ms(32);
	}
	// and calculate averages
	adc_gyrox_center  = adc_gyrox_center / 16;
	adc_gyroy_center  = adc_gyroy_center / 16;
	adc_accelx_center = adc_accelx_center / 16;
	adc_accely_center = adc_accely_center / 16;
	
	// and set the timing variables
	last_gyro_read = millis();
}

// set the ADC to run in either 8-bit mode (MODE_8_BIT) or 
// 10-bit mode (MODE_10_BIT)
void adc_setMode(uint8 mode)
{
	if (mode == MODE_10_BIT)
		ADMUX &= ~(1 << ADLAR);	// right-adjust result (ADC has result)
	else
		ADMUX |= 1 << ADLAR;	// left-adjust result (ADCH has result)	
}
	
// returns 0 if in 10-bit mode, otherwise returns non-zero.  The return
// value of this method can be directly compared against the macros
// MODE_8_BIT and MODE_10_BIT
uint8 adc_getMode()
{
	return (ADMUX >> ADLAR) & 1;
}

// returns the result of the previous ADC conversion.
uint16 adc_getConversionResult()
{
	if (adc_getMode())		// if left-adjusted (i.e. 8-bit mode)
	{
		return ADCH;		// 8-bit result
	} else {
		return ADC;			// 10-bit result
	}
}

// returns the result from the previous ADC conversion in millivolts.
uint16 adc_conversionResultMillivolts()
{
	if (adc_getMode())		 // if left-adjusted (i.e. 8-bit mode)
	{
		return adc_toMillivolts(ADCH);
	} else {
		return adc_toMillivolts(ADC);
	}
}

// converts the specified ADC result to millivolts
uint16 adc_toMillivolts(uint16 adcResult)
{
	unsigned long temp = adcResult * (unsigned long)millivolt_calibration;
	if (adc_getMode())				// if 8-bit mode
		return (temp + 127) / 255;
	return (temp + 511) / 1023;
}

// converts the specified ADC result to cm for the DMS sensor
uint8 adc_convertDMStoCM(uint16 adcResult)
{
	uint8 i = 0;
	// determine where the value fits
	for(i=0; (i<DMSTablePoints) && (adcResult>=(DMSTableValues[i])); i++);
	
	// check the value is inside the bounds
	if (i==0) {
		// distance is greater than 80cm, so we return 80
		return DMSTableCM[0];
	
	} else if ( i==DMSTablePoints ) {
		// distance is closer than 5cm (or value is invalid)
		return DMSTableCM[DMSTablePoints-1];
	
	} else {
		// find the closest value
		return DMSTableCM[i];
	}			
}

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
uint8 adc_isConverting()
{
	return (ADCSRA >> ADSC) & 1;
}
	
// The following function can be used to initiate an ADC conversion
// that runs in the background, allowing the CPU to perform other tasks
// while the conversion is in progress. The procedure is to start a
// conversion on a channel with startConversion(channel), and then
// poll isConverting in your main loop. Once isConverting() returns
// a zero, the result can be obtained through a call to conversionResult().
// Single conversion time is 120us (15 cycles at 125kHz)
void adc_startConversion(uint8 channel)
{
	// Channel numbers greater than 6 are invalid on the CM-510.
	if (channel > 6 && channel != 30)
	{
		return;
	}

	ADCSRA = 0x87;		// bit 7 set: ADC enabled
						// bit 6 clear: don't start conversion
						// bit 5 clear: disable autotrigger
						// bit 4: ADC interrupt flag
						// bit 3 clear: disable ADC interrupt
						// bits 0-2 set: ADC clock prescaler is 128
						// 128 prescaler required for 10-bit resolution when FCPU = 16 MHz
						
	// NOTE: it is important to make changes to a temporary variable and then set the ADMUX
	// register in a single atomic operation rather than incrementally changing bits of ADMUX.
	// Specifically, setting the ADC channel by first clearing the channel bits of ADMUX and
	// then setting the ones corresponding to the desired channel briefly connects the ADC
	// to channel 0, which can affect the ADC charge capacitor. 
	uint8 tempADMUX = ADMUX;

	tempADMUX |= 1 << 6;
	// use AVCC as a reference
	tempADMUX &= ~(1 << 7);

	tempADMUX &= ~0x1F;		 // clear channel selection bits of ADMUX
	tempADMUX |= channel;    // we only get this far if channel is less than 32
	ADMUX = tempADMUX;
	ADCSRA |= 1 << ADSC; // start the conversion
}

// take a single analog reading of the specified channel
uint16 adc_read(uint8 channel)
{
	adc_startConversion(channel);
	while (adc_isConverting());	// wait for conversion to finish
	return adc_getConversionResult();
}

// take a single analog reading of the specified channel and return the result in millivolts
uint16 adc_readMillivolts(uint8 channel)
{
	adc_startConversion(channel);
	while (adc_isConverting());	// wait for conversion to finish
	return adc_conversionResultMillivolts();
}

// take 'samples' readings of the specified channel and return the average
uint16 adc_readAverage(uint8 channel, uint16 samples)
{
	uint16 i = samples;
	unsigned long sum = 0;

	adc_startConversion(channel);	// call this first to set the channel
	while (adc_isConverting());		// wait while converting (discard first reading)
	do
	{
		ADCSRA |= 1 << ADSC;		// start the next conversion on current channel
		while (adc_isConverting());	// wait while converting
		sum += adc_getConversionResult();	// sum the results
	} while (--i);
	
	if (samples < 64)				// can do the division much faster
		return ((uint16)sum + (samples >> 1)) / (uint8)samples;
	return (sum + (samples >> 1)) / samples;	// compute the rounded avg
}

// take 'samples' readings of the specified channel and return the average in millivolts
uint16 adc_readAverageMillivolts(uint8 channel, uint16 samples)
{
	return adc_toMillivolts(adc_readAverage(channel, samples));
}

// sets the value used to calibrate the conversion from ADC reading
// to millivolts. The argument calibration should equal VCC in millivolts,
// which can be automatically measured using the function readVCCMillivolts():
// e.g. setMillivoltCalibration(readVCCMillivolts());
void adc_setMillivoltCalibration(uint16 calibration)
{
	millivolt_calibration = calibration;
}

// averages ten ADC readings of the fixed internal 1.1V bandgap voltage
// and computes VCC from the results.  This function returns VCC in millivolts.
// Channel 30 is the internal 1.1V BG on ATmega324/644/1284.
uint16 adc_readVCCMillivolts()
{
	uint8 mode = adc_getMode();
	adc_setMode(MODE_10_BIT);
	
	// bandgap cannot deliver much current, so it takes some time for the ADC
	// to settle to the BG voltage.  The following read connects the ADC to
	// the BG voltage and gives the voltage time to settle.
	adc_readAverage(30, 20);
	
	uint16 reading = adc_readAverage(30, 20);  // channel 30 is internal 1.1V BG
	uint16 value = (1023UL * 1100UL + (reading>>1)) / reading;
	adc_setMode(mode);
	return value;
}

// read battery voltage using 5 samples
// the CM-510 uses a resistive voltage divider that requires a factor 4 
uint16 adc_readBatteryMillivolts()
{
	// save current mode and set to 10-bit
	uint8 mode = adc_getMode();
	adc_setMode(MODE_10_BIT);
	
	// read battery for 5 conversion cycles and average
	uint16 value = adc_readAverageMillivolts(ADC_BATTERY, 5) * 4;
	
	// reset mode and return battery voltage
	adc_setMode(mode);
	return value;
}
