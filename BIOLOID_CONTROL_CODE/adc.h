/*
  ADC.h - Library for using the analog inputs on the Robotis CM-510
    controller. Reads ADC0 for battery voltage and ADC1-ADC6 from 
	the external 5-pin ports. By default ports are assigned as follows:
		GyroX = CM-510 Port3 = ADC3 = PORTF3
		GyroY = CM-510 Port4 = ADC4 = PORTF4
		DMS   = CM-510 Port5 = ADC5 = PORTF5
	Written by Peter Lanius
	Version 0.5		31/10/2011 
	Based on the Pololu library (see below)
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


#ifndef ADC_H_
#define ADC_H_

#define MODE_8_BIT		1
#define MODE_10_BIT		0


// initialization routine
void adc_init(void);

// function to process the sensor data when new data become available
// detects slips (robot has fallen over forward/backward)
// and also low battery alarms at this stage
// Returns:  int flag = 0 no new command
//           int flag = 1 new command
int adc_processSensorData();

// function that reads all the sensors from the main loop
// SENSOR_READ_INTERVAL in global.h determines how often the sensors are read
// BATTERY_READ_INTERVAL in global.h determines how often the battery voltage is read
// Returns:  int flag = 0 when no new values have been read
//           int flag = 1 when new values have been read
int adc_readSensors();

// set the ADC to 8 or 10 bit mode
// Input: Mode (MODE_8_BIT or MODE_10_BIT)
void adc_setMode(uint8 mode);

// get the ADC mode (8 or 10 bit)
// Returns: current ADC mode
uint8 adc_getMode();

// take a single analog reading of the specified channel
uint16 adc_read(uint8 channel);

// take a single analog reading of the specified channel and return result in millivolts
uint16 adc_readMillivolts(uint8 channel);

// take 'sample' readings of the specified channel and return the average
uint16 adc_readAverage(uint8 channel, uint16 samples);
									  
// take 'sample' readings of the specified channel and return the average in millivolts
uint16 adc_readAverageMillivolts(uint8 channel, uint16 samples);

// the following method can be used to initiate an ADC conversion
// that runs in the background, allowing the CPU to perform other tasks
// while the conversion is in progress.  The procedure is to start a
// conversion on a channel with startConversion(channel), and then
// poll isConverting in your main loop.  Once isConverting() returns
// a zero, the result can be obtained through a call to conversionResult().
void adc_startConversion(uint8 channel);

// returns 1 if the ADC is in the middle of an conversion, otherwise
// returns 0
uint8 adc_isConverting();
	
// returns the result of the previous ADC conversion.
uint16 adc_conversionResult();

// returns the result of the previous ADC conversion in millivolts.
uint16 adc_conversionResultMillivolts();
	
// sets the value used to calibrate the conversion from ADC reading
// to millivolts.  The argument calibration should equal VCC in millivolts,
// which can be automatically measured using the function readVCCMillivolts():
// e.g. setMillivoltCalibration(readVCCMillivolts());
void adc_setMillivoltCalibration(uint16 calibration);

// averages ten ADC readings of the fixed internal 1.1V bandgap voltage
// and computes VCC from the results.  This function returns VCC in millivolts.
// Channel 14 is internal 1.1V BG on ATmega48/168/328, but bit 5 of ADMUX is
// not used, so channel 30 is equivalent to channel 14.  Channel 30 is the internal
// 1.1V BG on ATmega324/644/1284.
uint16 adc_readVCCMillivolts();

// converts the specified ADC result to millivolts
uint16 adc_toMillivolts(uint16 adcResult);

// returns the voltage of the battery in millivolts using
// 10 averaged samples.
uint16 adc_readBatteryMillivolts();

// converts the specified ADC result to cm for the DMS sensor
uint8 adc_convertDMStoCM(uint16 adcResult);

#endif /* ADC_H_ */