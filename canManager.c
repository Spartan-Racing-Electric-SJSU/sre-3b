#include "IO_Driver.h" 
#include "IO_CAN.h"

#include "canManager.h"
#include "motorController.h"
#include "bms.h"

struct _CanManager {
	avlNode* canHistory;
	ubyte1 canHandle_HiPri_Read;
	ubyte1 canHandle_HiPri_Write;
	ubyte1 canHandle_LoPri_Read;
	ubyte1 canHandle_LoPri_Write;
};

//Keep track of CAN message IDs, their data, and when they were last sent.
struct _CanMessageNode
{
	IO_CAN_DATA_FRAME canMessage;
	ubyte4 timeBetweenMessages_Min;
	ubyte4 timeBetweenMessages_Max;
	ubyte1 lastMessage_data[8];
	ubyte4 lastMessage_timeStamp;
	canHistoryNode* left;
	canHistoryNode* right;
};

CanManager* CanManager_new()//ubyte4 defaultMinSendDelay, ubyte4 defaultMaxSendDelay)
{
	CanManager* me = (CanManager*)malloc(sizeof(struct _CanManager));

	//create can history data structure (AVL tree?)
	avlNode* canHistory = NULL;

	//do other can init stuff


	return me;
}

void CanManager_newMessageType(CanManager* me, ubyte2 messageID, ubyte4 timeBetweenMessages_Min, ubyte4 timeBetweenMessages_Max)
{
	me->canHistory = insert(me->canHistory, CanMessageNode* can);
}


IO_ErrorType CanManager_send(ubyte1 canHandle, IO_CAN_DATA_FRAME[] canMessages, ubyte ? canMessageCount, bool force);
//Check if message data has changed
//Check if it's time to send message yet
//If so, send message and update tree / linked list
//ToDo? write function to re-send message automatically if timebetweenMessages_Max has elapsed? (only applies to rinehart keepalive message)


//Helper functions
ubyte4 CanManager_timeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage)  //Overflows/resets at 74 min
bool CanManager_enoughTimeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) // timesincelast > timeBetweenMessages_Min
bool CanManager_dataChangedSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) //bitwise comparison for all data bytes



void canInput_readMessages(MotorController* mcm, BatteryManagementSystem* bms)
{
	//Read messages from hipri channel 
	ubyte1 messagesReceived;
	IO_CAN_ReadFIFO(canFifoHandle_HiPri_Read, canMessages, 25, &messagesReceived);
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
		case 0x620:
		case 0x621:
		case 0x622:
		case 0x623:
		case 0x624:
		case 0x625:
		case 0x626:
		case 0x627:
		case 0x628:
			//bms_parseCanMessage(bms, &canMessages[currMessage]);
			break;

			//default:

		}
	}

	//Echo message on lopri channel
	IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, messagesReceived);
	//IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
}
