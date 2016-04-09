#include <stdio.h>
#include "bms.h"
#include <stdlib.h>

/**************************************************************************
 * 	REVISION HISTORY:
 *
 *	2015-4-8 - Rabeel Elahi - Moved cases to canInput.c
							- Moved endian conversion functions to canInput.c
							- Changed uints to ubytes
							
 *	2016-4-7 - Rabeel Elahi - Created this file.
 *							- Defined cases for BMS can messages
 *							- TODO: Rename variable types to VCU types
 *							- TODO: Move canInput_readMesagges to caninput.c
 *							- TODO: Move ENDIAN conversion helper functions to caninput.c?
 *
 *
 **************************************************************************/

/**********************************************************
 *	 	 	 *********** CAUTION ***********              *
 * MULTI-BYTE VALUES FOR THE ELITHION BMS ARE BIG-ENDIAN  *
 *														  *
 **********************************************************/



struct _BMS{

	ubyte2 canMessageBaseId;

	// 0x622h //

	ubyte1  state; 			// state of system
	ubyte2  timer;				// power up time
	ubyte1  flags;				// flags
	ubyte1  faultCode; 		// fault code, stored
	ubyte1  levelFaults;		// Level fault flags (e.g. over voltage, under voltage, etc)
	ubyte1  warnings;			// warning flags

	// 0x623h //

	ubyte2 packVoltage;		// Total voltage of pack
	ubyte1  minVtg;			// Voltage of least charged cell
	ubyte1  minVtgCell;     	// ID of cell with lowest voltage
	ubyte1  maxVtg;			// Voltage of most charged cell
	ubyte1  maxVtgCell;     	// ID of cell with highest voltage

	// 0x624h //

	sbyte2  packCurrent;		// Pack current
	ubyte2  chargeLimit;		// Maximum current acceptable (charge)
	ubyte2  dischargeLimit;	// Maximum current available (discharge)

	// 0x625h //

	ubyte4 batteryEnergyIn; 	// Total energy into battery
	ubyte4 batteryEnergyOut;	// Total energy out of battery


	// 0x626h //

	ubyte1  SOC; 				// state of charge
	ubyte2 DOD; 				// depth of discharge
	ubyte2 capacity; 			// actual capacity of pack
	ubyte1  SOH;				// State of Health

	// 0x627h //

	sbyte1  packTemp;			// average pack temperature
	sbyte1  minTemp;			// Temperature of coldest sensor
	sbyte1  minTempCell; 		// ID of cell with lowest temperature
	sbyte1  maxTemp;			// Temperature of hottest sensor
	sbyte1  maxTempCell; 		// ID of cell with highest temperature

	// 0x628h //

	ubyte2 packRes;			// resistance of entire pack
	ubyte1  minRes;  			// resistance of lowest resistance cells
	ubyte1  minResCell;         // ID of cell with lowest resistance
	ubyte1  maxRes;				// resistance of highest resistance cells
	ubyte1  maxResCell;			// ID of cell with highest resistance



	// signed = 2's complement: 0XfFF = -1, 0x00 = 0, 0x01 = 1




};

BMS* BMS_new(int canMessageBaseID){

	BMS* BMS_obj = (BMS*)malloc(sizeof(struct _BMS));
	BMS_obj->canMessageBaseId = canMessageBaseID;
	return BMS_obj;

}







// ELITHION BMS OPTIONS //

//ubyte1  updateState(BMS*);
//ubyte2 updateTimer();
//ubyte1  updateFlags();
//ubyte1  updateFaultCode();
//ubyte1  updateLevelFaults();
//
//// PACK //
//
//ubyte2 updatePackVoltage(); 	// volts
//ubyte1  updateMinVtg(); 		// volts; individual cell voltage
//ubyte1  updateMaxVtg();
//ubyte1  updateMinVtgCell();
//ubyte1  updateMaxVtgCell();
//
//// CURRENT //
//
//sbyte2  updatePackCurrent(); 	 			// amps
//ubyte2 updateChargeLimit();				// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//ubyte2 updateDischargeLimit();			// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//
//ubyte1  updateSOC();			// Returns a value from 0-100
//ubyte2 updateDOD();		    // (Ah)
//ubyte2 updateCapacity();
//ubyte1  updateSOH();
//
//// TEMP //
//
//sbyte1  updatePackTemp();			// average pack temperature
//sbyte1  updateMinTemp();			    // Temperature of coldest sensor
//sbyte1  updateMinTempCell(); 		// ID of cell with lowest temperature
//sbyte1  updateMaxTemp();			    // Temperature of hottest sensor
//sbyte1  updateMaxTempCell(); 		// ID of cell with highest temperature
//
//ubyte2 updatePackRes();				// resistance of entire pack
//ubyte1  updateMinRes();  			// resistance of lowest resistance cells
//ubyte1  updateMinResCell();          // ID of cell with lowest resistance
//ubyte1  updateMaxRes();				// resistance of highest resistance cells
//ubyte1  updateMaxResCell();			// ID of cell with highest resistance
//
//
//LimitCause updateChargeLimitCause();
//LimitCause updateDischargeLimitCause();
//
//
////void getFaults(FaultOptions *presentFaults, StoredFault *storedFault, FaultOptions *presentWarnings);
//void clearStoredFault();
//
//IOFlags getIOFlags();


























