#include <stdio.h>
#include "bms.h"
#include <stdlib.h>
#include "IO_Driver.h"
#include "IO_RTC.h"

/**************************************************************************
 * 	REVISION HISTORY:
 *	2016-5-11 - Rabeel Elahi - Added bms_commands_getPower();
 *							 - Added bms_commands_getPackTemp();
 *							 
 *  2016-4-20 - Rabeel Elahi - Added bms_parseCANMessage()
 *						     - Moved cases back to bms.c
 *						     - Added #includes
 *						     - Leaving endian conversion functions until new CAN manager complete
 *
 * 	2016-4-8 - Rabeel Elahi - Moved cases to canInput.c
 *							- Moved endian conversion functions to canInput.c
 *							- Changed uints to ubytes
 *
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



struct _BatteryManagementSystem {

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

BatteryManagementSystem* BMS_new(ubyte2 canMessageBaseID){

	BatteryManagementSystem* BMS_obj = (BatteryManagementSystem*)malloc(sizeof(struct _BatteryManagementSystem));
	BMS_obj->canMessageBaseId = canMessageBaseID;
	return BMS_obj;

}

ubyte4 BMS_getPower(BatteryManagementSystem* me)
{
	return (me->packCurrent * me->packVoltage);
}

ubyte2 BMS_getPackTemp(BatteryManagementSystem* me)
{

	return (me->packTemp);
}


void BMS_parseCanMessage(BatteryManagementSystem* bms, IO_CAN_DATA_FRAME* bmsCanMessage){
	ubyte2 utemp16;
	sbyte1  temp16;
	ubyte4 utemp32;
	

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

		utemp32 = ((((ubyte4)bmsCanMessage->data[0] << 24) |
			((ubyte4)bmsCanMessage->data[1] << 16) |
			((ubyte4)bmsCanMessage->data[2] << 8) |
			(bmsCanMessage->data[3])));
		bms->batteryEnergyIn = swap_uint32(utemp32);

		utemp32 = ((((ubyte4)bmsCanMessage->data[4] << 24) |
			((ubyte4)bmsCanMessage->data[5] << 16) |
			((ubyte4)bmsCanMessage->data[6] << 8) |
			((ubyte4)bmsCanMessage->data[7])));
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

    case 0x629:
        //See https://onedrive.live.com/view.aspx?resid=F9BB8F0F8FDB5CF8!36803&ithint=file%2cxlsx&app=Excel&authkey=!AI-YHJrHmtUaWpI
        //Voltage(100mV)[022]
        //Current(100mA)[054]
        //Max Temp[104]
        //Avg Temp[096]
        //SOC(%)[112]
        //DOD(Ah)[144]
        break;
    }
}



ubyte1 swap_uint8(ubyte1 val)
{
	return (val << 4) | (val >> 4);
}

sbyte1 swap_int8(sbyte1 val)
{
	return (val << 4) | (val >> 4);
}
ubyte2 swap_uint16(ubyte2 val)
{
	return (val << 8) | (val >> 8 );
}

//! Byte swap short
sbyte2 swap_int16(sbyte2 val)
{
	return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned int
ubyte4 swap_uint32(ubyte4 val)
{
	val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
	return (val << 16) | (val >> 16);
}

//! Byte swap int
sbyte4 swap_int32(sbyte4 val)
{
	val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
	return (val << 16) | ((val >> 16) & 0xFFFF);
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


























