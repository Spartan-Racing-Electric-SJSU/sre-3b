#ifndef _BRAKEPRESSURESENSOR_H
#define _BRAKEPRESSURESENSOR_H

#include "IO_Driver.h"
#include "sensors.h"

/**
 * After update(), access to tps Sensor objects should no longer be necessary.
 * In other words, only updateFromSensors itself should use the tps Sensor objects
 * Also, all values in the TorqueEncoder object are from 
 */
typedef struct _BrakePressureSensor
{
    bool bench;					// Output from dev-board to indicate a testbench
	bool bps0_reverse;			// Brake Pressure Sensor 0 Reverse Flag
	bool runCalibration;		// Calibration In Progress Flag
	bool calibrated;			// Calibration Finished Flag
	bool implausibility;		// Disable cruise control/motor control when brakes applied

	ubyte1 brakePercentage;		// Amount of braking is applied
	ubyte1 calibrationRunTime;	// Duration of Brake Calibration

	ubyte2 bps0_calibMin;  		// 2 bytes for ADC, 4 bytes if switch to digital/timer/PWM
	ubyte2 bps0_calibMax;		// Maximum range of Brakes
	ubyte2 bps0_value;			// Hexadecimal input received from CAN Bus

	ubyte4 timestamp_calibrationStart;	// Real Time Clock Start Timer

	float4 percent;				// Final percentage value, or 0
    float4 bps0_percent;		// Calculated percentage from Calibration and BPS input

	Sensor* bps0;				// Raw ADC Input Data Structure

} BrakePressureSensor;

/**
 * Constructor for BPS struct
 */
BrakePressureSensor* BrakePressureSensor_new(void);
/**
 * Load new data for every struct member
 */
void BrakePressureSensor_update(BrakePressureSensor* me, bool bench);
/**
 * Gather raw sensor input from BPS0 and BPS1 
 */
void BrakePressureSensor_getIndividualSensorPercent(BrakePressureSensor* me, ubyte1 sensorNumber, float4* percent);
/**
 * Refreshes BPS values to constructor values
 */
void BrakePressureSensor_resetCalibration(BrakePressureSensor* me);
/**
 * Store calibration words into EEPROM
 */
void BrakePressureSensor_saveCalibrationToEEPROM(BrakePressureSensor* me);
/**
 * Load calibration words from EEPROM
 */
void BrakePressureSensor_loadCalibrationFromEEPROM(BrakePressureSensor* me);
/**
 * Initiate the calibration cycle for adjusting BPS values
 */
void BrakePressureSensor_startCalibration(BrakePressureSensor* me, ubyte1 secondsToRun);
/**
 * Iterate through calibration cycle for adjusting BPS values
 */
void BrakePressureSensor_calibrationCycle(BrakePressureSensor* me, ubyte1* errorCount);
/**
 * Retrieve the pedal travel distance calculation
 */
void BrakePressureSensor_getPedalTravel(BrakePressureSensor* me, ubyte1* errorCount, float4* pedalPercent);

#endif //  _BRAKEPRESSURESENSOR_H