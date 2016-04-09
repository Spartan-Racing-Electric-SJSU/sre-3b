#include "IO_Driver.h"
#include "IO_CAN.h"

#include "can.h"
#include "canInput.h"
#include "motorController.h"

//extern const ubyte1 canMessageLimit = 10;
extern IO_CAN_DATA_FRAME canMessages[];
ubyte2  utemp16;
sbyte2  temp16;
ubyte4  utemp32;

void canInput_readMessages(MotorController* mcm)
{
	//Read messages from hipri channel
	ubyte1 messagesReceived;
	IO_CAN_ReadFIFO(canFifoHandle_HiPri_Read, canMessages, 10, &messagesReceived);
	//IO_CAN_ReadFIFO(canFifoHandle_LoPri_Read, canMessages, 10, &messagesReceived);

	//IO_CAN_ReadMsg(canFifoHandle_HiPri_Read, canMessages);
	//messagesReceived = 1;

	//Determine message type based on ID
	for (int currMessage = 0; currMessage < messagesReceived; currMessage++)
	{
		switch (canMessages[currMessage].id)
		{
		//-------------------------------------------------------------------------
		//MCU Internal states
		//http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
		//-------------------------------------------------------------------------
		case 0xAA:
			//128 == 0x80 == 1 << 7 == the first bit is 1 and the rest are 0 == 10000000
			mcm_updateLockoutStatus(mcm, (canMessages[currMessage].data[6] & 0x80 == 0x80) ? ENABLED : DISABLED);
			mcm_updateInverterStatus(mcm, ((canMessages[currMessage].data[6] & 0x01 == 0x01) ? ENABLED : DISABLED));
			break;

			//-------------------------------------------------------------------------
			//MCU Fault codes
			//-------------------------------------------------------------------------
		case 0xAB:
			//For unknown/dynamic data array sizes, use dataIndex < sizeof(canMessages[currMessage].data) / sizeof(canMessages[currMessage].data[0]
			//for (int dataIndex = 0; dataIndex < 8; dataIndex++)
			//{
			//    switch (dataIndex)
			//    {
			//    case 6:   //Byte 6:
			//        break;
			//    default:
			//        break;
			//    }
			//}
			break;

			//-------------------------------------------------------------------------
			//BMS
			//-------------------------------------------------------------------------

		case 0x622:

			mcm->state = canMessages[currMessage].data[0];
			utemp16 = ((canMessages[currMessage].data[1] << 8) | (canMessages[currMessage].data[2]));
			mcm->timer = swap_uint16(temp);
			mcm->flags = canMessages[currMessage].data[3];
			mcm->faultCode = canMessages[currMessage].data[4];
			mcm->levelFaults = canMessages[currMessage].data[5];
			mcm->warnings = canMessages[currMessage].data[6]
			break;

		case 0x623:

			utemp16 = ((canMessages[currMessage].data[0] << 8) | (canMessages[currMessage].data[1]));
			mcm->packVoltage = swap_uint16(temp);
			mcm->minVtg = canMessages[currMessage].data[2];
			mcm->minVtgCell = canMessages[currMessage].data[3];
			mcm->maxVtg = canMessages[currMessage].data[4];
			mcm->maxVtgCell = canMessages[currMessage].data[5];
			break;

		case 0x624:

			temp16 = ((canMessages[currMessage].data[0] << 8) | (canMessages[currMessage].data[1]));
			mcm->packCurrent = swap_int16(temp16);

			utemp16 = ((canMessages[currMessage].data[2] << 8) | (canMessages[currMessage].data[3]));
			mcm->chargeLimit = swap_uint16(utemp16);

			utemp16 = ((canMessages[currMessage].data[4] << 8) | (canMessages[currMessage].data[5]));
			mcm->dischargeLimit = swap_uint16(utemp16);
			break;

		case 0x625:

			utemp32 = (((canMessages[currMessage].data[0] << 24) |
					(canMessages[currMessage].data[1] << 16) |
					(canMessages[currMessage].data[2] << 8) |
					(canMessages[currMessage].data[3])));
			mcm->batteryEnergyIn = swap_uint32(utemp32);

			utemp32 = (((canMessages[currMessage].data[4] << 24) |
					(canMessages[currMessage].data[5] << 16) |
					(canMessages[currMessage].data[6] << 8) |
					(canMessages[currMessage].data[7])));
			mcm->batteryEnergyOut = swap_uint32(utemp32);
			break;

		case 0x626:

			mcm->SOC = canMessages[currMessage].data[0];
			utemp16 = ((canMessages[currMessage].data[1] << 8) | (canMessages[currMessage].data[2]));
			mcm->DOD = swap_uint16(utemp16);

			utemp16 = ((canMessages[currMessage].data[3] << 8) | (canMessages[currMessage].data[4]));
			mcm->capacity = swap_uint16(utemp16);

			mcm->SOH = canMessages[currMessage].data[6];

			break;

		case 0x627:

			mcm->packTemp = canMessages[currMessage].data[0];
			mcm->minTemp = canMessages[currMessage].data[2];
			mcm->minTempCell = canMessages[currMessage].data[3];
			mcm->maxTemp = canMessages[currMessage].data[4];
			mcm->maxTempCell canMessages[currMessage].data[5];
			break;

		case 0x628:

			utemp16 = ((canMessages[currMessage].data[0] << 8) | (canMessages[currMessage].data[1]));
			mcm->packRes = swap_uint16(utemp16);

			mcm->minRes = canMessages[currMessage].data[2];
			mcm->minResCell = canMessages[currMessage].data[3];
			mcm->maxRes = canMessages[currMessage].data[4];
			mcm->maxResCell = canMessages[currMessage].data[5];
			break;
			//For unknown/dynamic data array sizes, use dataIndex < sizeof(canMessages[currMessage].data) / sizeof(canMessages[currMessage].data[0]
			//for (int dataIndex = 0; dataIndex < 8; dataIndex++)
			//{
			//    switch (dataIndex)
			//    {
			//    case 6:   //Byte 6:
			//        break;
			//    default:
			//        break;
			//    }
			//}

			//default:

		}
	}

	//Echo message on lopri channel
	IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, messagesReceived);
	//IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
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

