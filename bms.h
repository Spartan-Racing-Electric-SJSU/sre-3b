#include <stdio.h>
#include <stdint.h>


#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H


enum _FaultOptions
{
	DrivingOffWhilePluggedIn = 0x01,	// Driving off while plugged in
	InterlockTripped = 0x02,			// Inter-lock is tripped
	CommuncationFault = 0x04,			// Communication fault with a bank or cell
	ChargeOverCurrent = 0x08,			// Charge over-current
	DischargeOverCurrent = 0x10,        // Discharge over-current
	OverTemperture = 0x20,				// Over-temperature fault
	UnderVoltage = 0x40,				// Under voltage
	OverVoltage = 0x80,					// Over voltage

	// CUSTOM MESSAGES //

	BMSNotDetected = 0x100,
	InitFailed = 0x200,
};
typedef uint16_t FaultOptions; 			// FaultOptions is now a variable type -- uint16_t //


enum _StoredFaults
{
	StoredNoFault = 0x0,						// No fault
	StoredDrivingOffWhilePluggedIn = 0x01,		// Driving off while plugged in
	StoredInterockTripped = 0x02,				// Interlock is tripped
	StoredCommFault = 0x03,						// Communication fault with a bank or cell
	StoredChargeOverCurrent = 0x04,				// Charge over-current
	StoredDischargeOverCurrent = 0x05,			// Dishcarge over-current
	StoredOverTemperture = 0x06,				// Over-temperature fault
	StoredUnderVoltage = 0x07,					// Under voltage
	StoredOverVoltage = 0x08,					// Over voltage

	StoredNoBatteryVoltage = 0x09,              // No batter voltage
	StoredHighVoltageBMinusLeak = 0xA,			// High voltage B- leak to chassis
	StoredHighVoltageBPlusLeak = 0xB,			// High voltage B+ leak to chassis
	StoredContactorK1Shorted = 0xC,				// Contactor K1 shorted
	StoredContactorK2Shorted = 0xD,				// Contactor K2 shorted
	StoredContactorK3Shorted = 0xE,				// Contactor K3 shorted
	StoredNoPrecharge = 0xF,					// No precharge
	StoredOpenK2 = 0x10,						// Open K2
	StoredExcessivePrechargeTime = 0x11,		// Excessive precharge time
	StoredEEPROMStackOverflow = 0x12,			// EEPROM stack overflow

};

typedef uint8_t StoredFault;

enum _IOFlags{

	/*
	 * Using hex to represent each bit position
	 */

	IOFlagPowerFromSource = 0x01,				// There is power from the source
	IOFlagPowerFromLoad = 0x02,					// There is power from the load
	IOFlagInterlockedTripped = 0x04,			// The inter-lick is tripped
	IOFlagHardWireContactorRequest = 0x08,		// There is a hard-wire contactor request
	IOFlagCANContactorRequest = 0x10,			// There is a CAN contactor request
	IOFlagHighLimitSet = 0x20,					// The HLIM is set
	IOFlagLowLimitSet = 0x40,					// The LLIM is set
	IOFlagFanIsOn = 0x80,						// The fan is on

};

typedef uint8_t IOFlags;


enum _LimitCause{

	LimitCauseErrorReadingValue = -1,
	LimitCauseNone = 0,							// No limit
	LimitCausePackVoltageTooLow,				// Pack voltage too low
	LimitCausePackVolageTooHigh,				// Pack voltage too high
	LimitCauseCellVoltageTooLow,				// Cell voltage too low
	LimitCauseCellVoltageTooHigh,				// Cell voltage too high
	LimitCauseTempTooHighToCharge,				// Temperature too high for charging
	LimitCauseTempTooLowToCharge,				// Temperature too low for charging
	LimitCauseTempTooHighToDischarge,			// Temperature too high for discharging
	LimitCauseTempTooLowToDischarge,			// Temperature too low for discharging
	LimitCauseChargingCurrentPeakTooLong,		// Charging current peak lasting too long
	LimitCauseDischargingCurrentPeakTooLong,	// Discharging current peak lasted too long
};

typedef int8_t LimitCause;

#define ERROR_READING_LIMIT_VALUE = -1






typedef struct _BMS *BMS;

	BMS newBMSObj();

	// ELITHION BMS OPTIONS //

	uint8_t getStateOfCharge();			// Returns a value from 0-100
	uint16_t getDepthOfDischarge();		// (Ah)

	LimitCause getChargeLimitCause();
	LimitCause getDischargeLimitCause();

	int8_t getChargeLimitValue(); 		// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
	int8_t getDishargeLimitValue(); 	// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error

	// PACK //

	float getPackVoltage(); 	// volts
	float getMinVolage(); 		// volts; individual cell voltage
	float getAvgVoltage();
	float getMaxVoltage();
	uint8_t getMinVoltageCellNumber();
	uint8_t getAvgVoltageCellNumber();
	uint8_t getMaxVoltageCellNUmber();

	int getNumberOfCells();
	float getVoltageForCell(int cell);

	// CURRENT //

	float getPackCurrent(); 	 			// amps
	float getAverageSourceCurrent();		// amps
	float getAverageLoadCurrent(); 			// amps
	float getSourceCurrent(); 				// amps
	float getLoadCurrent(); 				// amps

	void getFaults(FaultOptions *presentFaults, StoredFault *storedFault, FaultOptions *presentWarnings);
	void clearStoredFault();

	IOFlags getIOFlags();



#endif // _BATTERYMANAGEMENTSYSTEM_H

//
//	typedef struct _BatteryManagementSystem BatteryManagementSystem;
//
//	BatteryManagementSystem* bms_new(int canMessageBaseID); // initializing other variables?
//
//	void bms_updateFromCAN(int* CAN_Message);
//	//
//	// recieving message from CANinput function
//	// taking message and im storing into variables so they can be used later and can be
//	//



