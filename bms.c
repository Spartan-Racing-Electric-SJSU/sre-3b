#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_CAN.h"

/**************************************************************************
* 	REVISION HISTORY:
*
*
*	2016-4-7 - Rabeel Elahi - Created this file.
*							- Defined cases for BMS can messages
*							- Helper functions commented out; 
*							  directly modifying variables via ptrs instead
*							- TODO: Rename variable types to VCU types
*							- TODO: Move canInput_readMesagges to caninput.c
*							- TODO: Move ENDIAN conversion functions to caninput.c?
*
*
**************************************************************************/

/**********************************************************
 *	 	 	 *********** CAUTION ***********              *
 * MULTI-BYTE VALUES FOR THE ELITHION BMS ARE BIG-ENDIAN  *
 *														  *
 **********************************************************/



struct _BatteryManagementSystem {

	uint16_t canMessageBaseId;

	// 0x622h //

	uint8_t  state; 			// state of system
	uint16_t timer;				// power up time
	uint8_t  flags;				// flags
	uint8_t  faultCode; 		// fault code, stored
	uint8_t  levelFaults;		// Level fault flags (e.g. over voltage, under voltage, etc)
	uint8_t  warnings;			// warning flags

	// 0x623h //

	uint16_t packVoltage;		// Total voltage of pack
	uint8_t  minVtg;			// Voltage of least charged cell
	uint8_t  minVtgCell;     	// ID of cell with lowest voltage
	uint8_t  maxVtg;			// Voltage of most charged cell
	uint8_t  maxVtgCell;     	// ID of cell with highest voltage

	// 0x624h //

	int16_t  packCurrent;		// Pack current
	uint16_t chargeLimit;		// Maximum current acceptable (charge)
	uint16_t dischargeLimit;	// Maximum current available (discharge)

	// 0x625h //

	uint32_t batteryEnergyIn; 	// Total energy into battery
	uint32_t batteryEnergyOut;	// Total energy out of battery


	// 0x626h //

	uint8_t  SOC; 				// state of charge
	uint16_t DOD; 				// depth of discharge
	uint16_t capacity; 			// actual capacity of pack
	uint8_t  SOH;				// State of Health

	// 0x627h //

	int8_t  packTemp;			// average pack temperature
	int8_t  minTemp;			// Temperature of coldest sensor
	int8_t  minTempCell; 		// ID of cell with lowest temperature
	int8_t  maxTemp;			// Temperature of hottest sensor
	int8_t  maxTempCell; 		// ID of cell with highest temperature

	// 0x628h //

	uint16_t packRes;			// resistance of entire pack
	uint8_t  minRes;  			// resistance of lowest resistance cells
	uint8_t  minResCell;         // ID of cell with lowest resistance
	uint8_t  maxRes;				// resistance of highest resistance cells
	uint8_t  maxResCell;			// ID of cell with highest resistance



	// signed = 2's complement: 0XfFF = -1, 0x00 = 0, 0x01 = 1




};

BatteryManagementSystem* bms_new(int canMessageBaseID) {

	BatteryManagementSystem* BMS_obj = (BatteryManagementSystem*)malloc(sizeof(struct _BatteryManagementSystem));
	BMS_obj->canMessageBaseId = canMessageBaseID;
	return BMS_obj;

}



/*
 * TODO: change variables to VCU type variables
 * TODO: move function below to caninput.c
 *
 * NOTE: MULTI-BYTE VALUES ARE BIG ENDIAN!
 */
void bms_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage){
	uint16_t utemp16;
	int16_t  temp16;
	uint32_t utemp32;

	switch (bmsCanMessage->id)
	{

	case 0x622:

		bms->state = bmsCanMessage->data[0];
		utemp16 = ((bmsCanMessage->data[1] << 8) | (bmsCanMessage->data[2]));
		bms->timer = swap_uint16(utemp16);
		bms->flags = bmsCanMessage->data[3];
		bms->faultCode = bmsCanMessage->data[4];
		bms->levelFaults = bmsCanMessage->data[5];
		bms->warnings = bmsCanMessage->data[6];

		break;

	case 0x623:

		utemp16 = ((bmsCanMessage->data[0] << 8) | (bmsCanMessage->data[1]));
		bms->packVoltage = swap_uint16(utemp16);
		bms->minVtg = bmsCanMessage->data[2];
		bms->minVtgCell = bmsCanMessage->data[3];
		bms->maxVtg = bmsCanMessage->data[4];
		bms->maxVtgCell = bmsCanMessage->data[5];

		break;

	case 0x624:

		temp16 = ((bmsCanMessage->data[0] << 8) | (bmsCanMessage->data[1]));
		bms->packCurrent = swap_int16(temp16);

		utemp16 = ((bmsCanMessage->data[2] << 8) | (bmsCanMessage->data[3]));
		bms->chargeLimit = swap_uint16(utemp16);

		utemp16 = ((bmsCanMessage->data[4] << 8) | (bmsCanMessage->data[5]));
		bms->dischargeLimit = swap_uint16(utemp16);

		break;

	case 0x625:

		utemp32 = (((bmsCanMessage->data[0] << 24) |
			(bmsCanMessage->data[1] << 16) |
			(bmsCanMessage->data[2] << 8) |
			(bmsCanMessage->data[3])));
		bms->batteryEnergyIn = swap_uint32(utemp32);

		utemp32 = (((bmsCanMessage->data[4] << 24) |
			(bmsCanMessage->data[5] << 16) |
			(bmsCanMessage->data[6] << 8) |
			(bmsCanMessage->data[7])));
		bms->batteryEnergyOut = swap_uint32(utemp32);

		break;

	case 0x626:

		bms->SOC = bmsCanMessage->data[0];

		utemp16 = ((bmsCanMessage->data[1] << 8) | (bmsCanMessage->data[2]));
		bms->DOD = swap_uint16(utemp16);

		utemp16 = ((bmsCanMessage->data[3] << 8) | (bmsCanMessage->data[4]));
		bms->capacity = swap_uint16(utemp16);

		bms->SOH = bmsCanMessage->data[6];

		break;

	case 0x627:

		bms->packTemp = bmsCanMessage->data[0];
		bms->minTemp = bmsCanMessage->data[2];
		bms->minTempCell = bmsCanMessage->data[3];
		bms->maxTemp = bmsCanMessage->data[4];
		bms->maxTempCell = bmsCanMessage->data[5];

		break;

	case 0x628:

		utemp16 = ((bmsCanMessage->data[0] << 8) | (bmsCanMessage->data[1]));
		bms->packRes = swap_uint16(utemp16);

		bms->minRes = bmsCanMessage->data[2];
		bms->minResCell = bmsCanMessage->data[3];
		bms->maxRes = bmsCanMessage->data[4];
		bms->maxResCell = bmsCanMessage->data[5];

		break;

	}
}



uint8_t swap_uint8(uint8_t val)
{
	return (val << 4) | (val >> 4);
}

int8_t swap_int8(int8_t val)
{
	return (val << 4) | (val >> 4);
}
uint16_t swap_uint16(uint16_t val)
{
	return (val << 8) | (val >> 8 );
}

//! Byte swap short
int16_t swap_int16(int16_t val)
{
	return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned int
uint32_t swap_uint32(uint32_t val)
{
	val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
	return (val << 16) | (val >> 16);
}

//! Byte swap int
int32_t swap_int32(int32_t val)
{
	val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
	return (val << 16) | ((val >> 16) & 0xFFFF);
}




// ELITHION BMS OPTIONS //

//uint8_t  updateState(BMS*);
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
//// CURRENT //
//
//int16_t  updatePackCurrent(); 	 			// amps
//uint16_t updateChargeLimit();				// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//uint16_t updateDischargeLimit();			// 0-100 percent; returns EROR_READING_LIMIT_VALUE on error
//
//uint8_t  updateSOC();			// Returns a value from 0-100
//uint16_t updateDOD();		    // (Ah)
//uint16_t updateCapacity();
//uint8_t  updateSOH();
//
//// TEMP //
//
//int8_t  updatePackTemp();			// average pack temperature
//int8_t  updateMinTemp();			    // Temperature of coldest sensor
//int8_t  updateMinTempCell(); 		// ID of cell with lowest temperature
//int8_t  updateMaxTemp();			    // Temperature of hottest sensor
//int8_t  updateMaxTempCell(); 		// ID of cell with highest temperature
//
//uint16_t updatePackRes();				// resistance of entire pack
//uint8_t  updateMinRes();  			// resistance of lowest resistance cells
//uint8_t  updateMinResCell();          // ID of cell with lowest resistance
//uint8_t  updateMaxRes();				// resistance of highest resistance cells
//uint8_t  updateMaxResCell();			// ID of cell with highest resistance
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


























