 /**************************************************************************
 * 	REVISION HISTORY:
 *
 *
 *	2016-4-6  - Rabeel Elahi - Added constructor and BMS data struct
 *							 - Initially added helper functions to update variables,
 *							   but decided to update variables by passing BMS pointer to
 *							   canInput_readMesagges(BMS* bms). Commented out for future use.
 *							 - Added functions to help with endian conversion.
 *							   Possibly move to mathFunctions?
 *
 *	2016-3-28 - Rabeel Elahi - Created this file.
 *
 *
 **************************************************************************/


#include <stdio.h>
#include <stdint.h>
#include "IO_CAN.h"


#ifndef _BATTERYMANAGEMENTSYSTEM_H
#define _BATTERYMANAGEMENTSYSTEM_H

typedef struct _BatteryManagementSystem BatteryManagementSystem;

BatteryManagementSystem* bms_new(int canMessageBaseID);
void bms_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage);

/*
 *  Functions for endian conversion
 */
uint8_t swap_uint8(uint8_t val);
int8_t swap_int8(int8_t val);
uint16_t swap_uint16(uint16_t val);
int16_t swap_int16(int16_t val);
uint32_t swap_uint32(uint32_t val);
int32_t swap_int32(int32_t val);

typedef enum
{
	relayFault = 0x08,
	contactorK3Status = 0x04,
	contactorK2Status = 0x02,
	contactorK1Status = 0x01,
	faultState = 0x00,

} systemState;

typedef enum
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

} faultOptions;



typedef enum
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

	StoredNoBatteryVoltage = 0x09,              // No battery voltage
	StoredHighVoltageBMinusLeak = 0xA,			// High voltage B- leak to chassis
	StoredHighVoltageBPlusLeak = 0xB,			// High voltage B+ leak to chassis
	StoredContactorK1Shorted = 0xC,				// Contactor K1 shorted
	StoredContactorK2Shorted = 0xD,				// Contactor K2 shorted
	StoredContactorK3Shorted = 0xE,				// Contactor K3 shorted
	StoredNoPrecharge = 0xF,					// No precharge
	StoredOpenK2 = 0x10,						// Open K2
	StoredExcessivePrechargeTime = 0x11,		// Excessive precharge time
	StoredEEPROMStackOverflow = 0x12,			// EEPROM stack overflow

} storedFaults;



typedef enum {

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

} IOFlags;




typedef enum LimitCause{

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

} LimitCause;

#define ERROR_READING_LIMIT_VALUE = -1






//
//// ELITHION BMS OPTIONS //
//
//uint8_t  updateState();
//uint16_t updateTimer();
//uint8_t  updateFlags();
//uint8_t  updateFaultCode();
//uint8_t  updateLevelFaults();
//
//// PACK //
//
//uint16_t updatePackVoltage(); 	// volts
//uint8_t  updateMinVtg(); 		// volts; individual cell voltage
//uint8_t  updateMaxVtg();
//uint8_t  updateMinVtgCell();
//uint8_t  updateMaxVtgCell();
//
//
//// CURRENT //
//
//int16_t  updatePackCurrent(); 	 			// amps
//uint16_t updateChargeLimit();				// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//uint16_t updateDischargeLimit();			// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//
//// BATTERY //
//
//uint32_t batteryEnergyIn();
//uint32_t batteryEnergyOut();
//
//
//uint8_t  updateSOC();
//uint16_t updateDOD();
//uint16_t updateCapacity();
//uint8_t  updateSOH();
//
//// TEMP //
//
//int8_t  updatePackTemp();			     // average pack temperature
//int8_t  updateMinTemp();			     // Temperature of coldest sensor
//int8_t  updateMinTempCell(); 		     // ID of cell with lowest temperature
//int8_t  updateMaxTemp();			     // Temperature of hottest sensor
//int8_t  updateMaxTempCell(); 		     // ID of cell with highest temperature
//
//
//// RESISTANCE //
//
//uint16_t updatePackRes();				// resistance of entire pack
//uint8_t  updateMinRes();  			// resistance of lowest resistance cells
//uint8_t  updateMinResCell();          // ID of cell with lowest resistance
//uint8_t  updateMaxRes();				// resistance of highest resistance cells
//uint8_t  updateMaxResCell();			// ID of cell with highest resistance
//
//LimitCause updateChargeLimitCause();
//LimitCause updateDischargeLimitCause();

//
////void getFaults(FaultOptions *presentFaults, StoredFault *storedFault, FaultOptions *presentWarnings);
//void clearStoredFault();
//
//IOFlags getIOFlags();
//


#endif // _BATTERYMANAGEMENTSYSTEM_H



