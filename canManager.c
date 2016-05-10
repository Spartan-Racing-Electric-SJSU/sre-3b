
#include <stdlib.h> //malloc

#include "IO_Driver.h" 
#include "IO_CAN.h"
#include "IO_RTC.h"

#include "canManager.h"
#include "avlTree.h"
#include "motorController.h"
#include "bms.h"

struct _CanManager {
    AVLNode* incomingTree;
    AVLNode* outgoingTree;

    ubyte1 canMessageLimit;
	
    //These are our four FIFO queues.  All messages should come/go through one of these queues.
    //Functions shall have a CanChannel enum (see header) parameter.  Direction (send/receive is not
    //specified by this parameter.  The CAN0/CAN1 is selected based on the parameter passed in, and 
    //Read/Write is selected based on the function that is being called (get/send)
    ubyte1 can0_busSpeed;
    ubyte1 can0_readHandle;
    ubyte1 can0_read_messageLimit;
    ubyte1 can0_writeHandle;
    ubyte1 can0_write_messageLimit;

    ubyte1 can1_busSpeed;
    ubyte1 can1_readHandle;
    ubyte1 can1_read_messageLimit;
    ubyte1 can1_writeHandle;
    ubyte1 can1_write_messageLimit;
    
    IO_ErrorType ioErr_can0_Init;
    IO_ErrorType ioErr_can1_Init;

    IO_ErrorType ioErr_can0_fifoInit_R;
    IO_ErrorType ioErr_can0_fifoInit_W;
    IO_ErrorType ioErr_can1_fifoInit_R;
    IO_ErrorType ioErr_can1_fifoInit_W;

    IO_ErrorType ioErr_can0_read;
    IO_ErrorType ioErr_can0_write;
    IO_ErrorType ioErr_can1_read;
    IO_ErrorType ioErr_can1_write;

    ubyte2 sendDelayMs;


    IO_CAN_DATA_FRAME canMessages[];//Flexible array must be the last struct member
};

//Keep track of CAN message IDs, their data, and when they were last sent.
/*
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
*/

CanManager* CanManager_new(ubyte2 can0_busSpeed, ubyte1 can0_read_messageLimit, ubyte1 can0_write_messageLimit
                         , ubyte2 can1_busSpeed, ubyte1 can1_read_messageLimit, ubyte1 can1_write_messageLimit
                         , ubyte2 defaultSendDelayMs) //ubyte4 defaultMinSendDelay, ubyte4 defaultMaxSendDelay)
{
	CanManager* me = (CanManager*)malloc(sizeof(struct _CanManager));

	//create can history data structure (AVL tree?)
	me->incomingTree = NULL;
    me->outgoingTree = NULL;
    me->sendDelayMs = defaultSendDelayMs;

    //Activate the CAN channels --------------------------------------------------
    me->ioErr_can0_Init = IO_CAN_Init(IO_CAN_CHANNEL_0, can0_busSpeed, 0, 0, 0);
    me->ioErr_can1_Init = IO_CAN_Init(IO_CAN_CHANNEL_1, can1_busSpeed, 0, 0, 0);

    //Configure the FIFO queues
    //This specifies: The handle names for the queues
    //, which channel the queue belongs to
    //, the # of messages (or maximum count?)
    //, the direction of the queue (in/out)
    //, the frame size
    //, and other stuff?
    IO_CAN_ConfigFIFO(&me->can0_readHandle, IO_CAN_CHANNEL_0, can0_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can0_writeHandle, IO_CAN_CHANNEL_0, can0_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can1_readHandle, IO_CAN_CHANNEL_1, can1_read_messageLimit, IO_CAN_MSG_READ, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&me->can1_writeHandle, IO_CAN_CHANNEL_1, can1_write_messageLimit, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

    //Assume read/write at error state until used
    me->ioErr_can0_read = IO_E_CAN_BUS_OFF;
    me->ioErr_can0_write = IO_E_CAN_BUS_OFF;
    me->ioErr_can1_read = IO_E_CAN_BUS_OFF;
    me->ioErr_can1_write = IO_E_CAN_BUS_OFF;

    //-------------------------------------------------------------------
    //Define default messages
    //-------------------------------------------------------------------
    //Incoming
    AVL_insert(me->outgoingTree, 0x0C0, NULL, 10, 125, TRUE); //MCM command message

    //Outgoing
    AVL_insert(me->incomingTree, 0X0AA, NULL, 0, 10000, TRUE); //MCM Status
    AVL_insert(me->incomingTree, 0X0AB, NULL, 0, 10000, TRUE); //MCM Faults
    AVL_insert(me->incomingTree, 0x623, NULL, 0, 5000, TRUE); //BMS faults
    AVL_insert(me->incomingTree, 0x629, NULL, 0, 1000, TRUE); //BMS high priority message


	return me;
}


/*****************************************************************************
* This function takes an array of messages, determines which messages to send
* based on whether or not data has changed since the last time it was sent,
* or if a certain amount of time has passed since the last time it was sent.
*
* Messages that need to be sent are copied to another array and passed to the
* FIFO queue.
*
* Note: http://stackoverflow.com/questions/5573310/difference-between-passing-array-and-array-pointer-into-function-in-c
* http://stackoverflow.com/questions/2360794/how-to-pass-an-array-of-struct-using-pointer-in-c-c
****************************************************************************/
IO_ErrorType CanManager_send(CanManager* me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount)
{
    bool sendMessage = FALSE;
    ubyte1 messagesToSendCount = 0;
    IO_CAN_DATA_FRAME messagesToSend[canMessageCount];  //The biggest possible array size would be if every message needs to be sent.

    //----------------------------------------------------------------------------
    // Check if message exists in outgoing message history tree
    //----------------------------------------------------------------------------
    AVLNode* lastMessage;
    ubyte1 messagePosition; //used twice
    for (messagePosition = 0; messagePosition < canMessageCount; messagePosition++)
    {
        //NEEDS TO USE DIFFERENT TREES FOR CAN0 / CAN1
        //lastMessage = AVL_find(me->outgoingTree, canMessages[messagePosition]->id);
        lastMessage = AVL_find(me->outgoingTree, canMessages[messagePosition].id);

        // Message doesn't exist in history tree ---------------------------------
        if (lastMessage == NULL)
        {
            lastMessage = AVL_insert(me->outgoingTree, (canMessages[messagePosition]).id, canMessages[messagePosition].data, me->sendDelayMs, me->sendDelayMs, FALSE);
            //send message later
            sendMessage = TRUE;
        }
        // This message exists in the history tree ---------------------------------
        else
        {
            //----------------------------------------------------------------------------
            // Check if data has changed since last time message was sent
            //----------------------------------------------------------------------------
            bool dataChanged = FALSE;
            for (ubyte1 dataPosition = 0; dataPosition < 8; dataPosition++)
            {
                ubyte1 oldData = lastMessage->data[dataPosition];
                ubyte1 newData = canMessages[messagePosition].data[dataPosition];
                //if any data byte is changed, then probably want to send the message
                if (oldData == newData)
                {
                    //data is the same
                }
                else
                {
                    dataChanged = TRUE;
                }
            }
            
            //----------------------------------------------------------------------------
            // If data has changed
            //----------------------------------------------------------------------------
            if (dataChanged == TRUE)
            {
                //Send message if >10ms have passed (or whatever the min time is for this message)
                if (IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= lastMessage->timeBetweenMessages_Min)
                {
                    sendMessage = TRUE;
                }
            }
            //----------------------------------------------------------------------------
            // If data has NOT changed
            //----------------------------------------------------------------------------
            else
            {
                //If it's been a long time since this message was last sent out
                if (IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp) >= lastMessage->timeBetweenMessages_Max)
                {
                    //Send the message even though the data hasn't changed
                    sendMessage = TRUE;
                }
            }
        }
        //----------------------------------------------------------------------------
        // end check if message should be sent
        //----------------------------------------------------------------------------

        //----------------------------------------------------------------------------
        // If we determined that this message should be sent
        //----------------------------------------------------------------------------
        if (sendMessage == TRUE)
        {
            //copy the message that needs to be sent into the outgoing messages array
            //see http://stackoverflow.com/questions/1693853/copying-arrays-of-structs-in-c
            //http://www.socialledge.com/sjsu/index.php?title=ES101_-_Lesson_9_:_Structures
            messagesToSend[messagesToSendCount++] = canMessages[messagePosition];
        }
    } //end of loop for each message in outgoing messages

    //----------------------------------------------------------------------------
    // If there are messages to send
    //----------------------------------------------------------------------------
    IO_ErrorType sendResult = IO_E_OK;
    if (messagesToSendCount > 0)
    {
        //Send the messages to send to the appropriate FIFO queue
        sendResult = IO_CAN_WriteFIFO((channel == CAN0_HIPRI) ? me->can0_writeHandle : me->can1_writeHandle, messagesToSend, canMessageCount);
        *((channel == CAN0_HIPRI) ? &me->ioErr_can0_write : &me->ioErr_can1_write) = sendResult;

        //Update the outgoing message tree with message sent timestamps
        if ((channel == CAN0_HIPRI ? me->ioErr_can0_write : me->ioErr_can1_write) == IO_E_OK)
        {
            //Loop through the messages that we sent...
            AVLNode* messageToUpdate;
            for (messagePosition = 0; messagePosition < messagesToSendCount; messagePosition++)
            {
                //...find the message ID in the outgoing message tree (AGAIN - big inefficiency here)...
                messageToUpdate = AVL_find(me->outgoingTree, messagesToSend[messagePosition].id);

                //and update the message sent timestamp
                IO_RTC_GetTimeUS(messageToUpdate->lastMessage_timeStamp); //Update the timestamp for when the message was last sent
            }
        }
    }

    return sendResult;
}

/*
//Helper functions
ubyte4 CanManager_timeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage)  //Overflows/resets at 74 min
bool CanManager_enoughTimeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) // timesincelast > timeBetweenMessages_Min
bool CanManager_dataChangedSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) //bitwise comparison for all data bytes
*/


/*****************************************************************************
* read
****************************************************************************/
void CanManager_read(CanManager* me, CanChannel channel, MotorController* mcm, BatteryManagementSystem* bms)
{
    IO_CAN_DATA_FRAME canMessages[(channel == CAN0_HIPRI ? me->can0_read_messageLimit : me->can1_read_messageLimit)];
    ubyte1 canMessageCount;  //FIFO queue only holds 128 messages max

	//Read messages from hipri channel 
	*(channel == CAN0_HIPRI ? &me->ioErr_can0_read : &me->ioErr_can1_read) =
    IO_CAN_ReadFIFO((channel == CAN0_HIPRI ? me->can0_readHandle : me->can1_writeHandle)
                    , canMessages
                    , (channel == CAN0_HIPRI ? me->can0_read_messageLimit : me->can1_read_messageLimit)
                    , &canMessageCount);

	//Determine message type based on ID
	for (int currMessage = 0; currMessage < canMessageCount; currMessage++)
	{
		switch (canMessages[currMessage].id)
		{
        //-------------------------------------------------------------------------
        //Motor controller
        //-------------------------------------------------------------------------
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8:
        case 0xA9:
            break;
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

        case 0xAC:
        case 0xAD:
        case 0xAE:
        case 0xAF:
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
	//IO_CAN_WriteFIFO(me->can1_writeHandle, canMessages, messagesReceived);
    CanManager_send(me, CAN1_LOPRI, canMessages, canMessageCount);
    //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
}

/*****************************************************************************
* device-specific functions
****************************************************************************/
/*****************************************************************************
* Standalone Sensor messages
******************************************************************************
* Load sensor values into CAN messages
* Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
* The message addresses are at:
* https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
****************************************************************************/
void canOutput_sendSensorMessages(CanManager* me)
{

}


/*****************************************************************************
* Motor Controller (MCU) control message
******************************************************************************
* This function builds a IO_CAN_DATA_FRAME (can message) based on
* the data that needs to be sent to the motor controller,
* then pass the IO_CAN_DATA_FRAME to the CanManager via CanManager_Send()
****************************************************************************/
void MotorController_sendControlMessage(MotorController* me, CanManager* canMan, bool sendEvenIfNoChanges)
{
    //Only send a message if there's an update or it's been > .25 seconds or force=true
    IO_CAN_DATA_FRAME canMessage;
    //Rinehart CAN control message (heartbeat) structure ----------------
        canMessages[0].length = 8; // how many bytes in the message
        canMessages[0].id_format = IO_CAN_STD_FRAME;
        canMessages[0].id = 0xC0;

        //Torque (Nm * 10)
        ubyte2 mcuTorque = 5; //In Nm * 10. 125 continuous, 240 max
        canMessages[0].data[0] = (ubyte1)mcm_commands_getTorque(mcm);
        canMessages[0].data[1] = mcm_commands_getTorque(mcm) >> 8;

        //Speed (RPM?) - not needed - mcu should be in torque mode
        canMessages[0].data[2] = 0;
        canMessages[0].data[3] = 0;

        //Direction: 0=CW, 1=CCW
        canMessages[0].data[4] = mcm_commands_getDirection(mcm);

        //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
        canMessages[0].data[5] = 0; //First set whole byte to zero

                                    //Next add each bit one at a time, starting with the bit that belongs in the leftmost position
        for (int bit = 7; bit >= 0; bit--)
        {
            canMessages[0].data[5] <<= 1;  //Always leftshift first
            switch (bit)
            {
                // Then add your bit to the right (note: the order of case statements doesn't matter - it's the fact that bit-- instead of bit++;)
            case 1: canMessages[0].data[5] |= (mcm_commands_getDischarge(mcm) == ENABLED) ? 1 : 0; break;
            case 0: canMessages[0].data[5] |= (mcm_commands_getInverter(mcm) == ENABLED) ? 1 : 0; break;  // Then add your bit to the right

            }
        }

        //Unused (future use)
        canMessages[0].data[6] = 0;
        canMessages[0].data[7] = 0;


        //Place the can messsages into the FIFO queue ---------------------------------------------------
        //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, 1);  //Important: Only transmit one message (the MCU message)
        //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, 1);  //Important: Only transmit one message (the MCU message)
        CanManager_send(canMan, CAN0_HIPRI, canMessages, 1);
        CanManager_send(canMan, CAN1_LOPRI, canMessages, 1);

        //Reset the last message count/timestamp
        mcm_commands_resetUpdateCountAndTime(mcm);
        //IO_RTC_StartTime(&mcm.commands.timeStamp_lastCommandSent);
        //mcm.commands.updateCount = 0;

        //IO_CAN_WriteMsg(canFifoHandle_HiPri_Write, &canMessages);  //Important: Only transmit one message (the MCU message)
        //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, &canMessages);  //Important: Only transmit one message (the MCU message)

}


//----------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------
void canOutput_sendDebugMessage(CanManager* me, TorqueEncoder* tps, BrakePressureSensor* bps, MotorController* mcm, WheelSpeeds* wss, SafetyChecker* sc)
{
    ubyte1 errorCount;
    float4 tempPedalPercent;   //Pedal percent float (a decimal between 0 and 1
    ubyte2 tps0Percent;  //Pedal percent int   (a number from 0 to 100)
    ubyte2 tps1Percent;
    ubyte2 canMessageCount = 0;
    ubyte2 canMessageID = 0x500;
    ubyte1 byteNum;

    TorqueEncoder_getIndividualSensorPercent(tps, 0, &tempPedalPercent); //borrow the pedal percent variable
    tps0Percent = 0xFF * tempPedalPercent;
    TorqueEncoder_getIndividualSensorPercent(tps, 1, &tempPedalPercent);
    tps1Percent = 0xFF * (1 - tempPedalPercent);

    TorqueEncoder_getPedalTravel(tps, &errorCount, &tempPedalPercent); //getThrottlePercent(TRUE, &errorCount);
    ubyte2 throttlePercent = 0xFF * tempPedalPercent;

    BrakePressureSensor_getPedalTravel(bps, &errorCount, &tempPedalPercent); //getThrottlePercent(TRUE, &errorCount);
    ubyte2 brakePercent = 0xFF * tempPedalPercent;

    /*
    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = throttlePercent; //mcm_getStartupStage(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = tps1Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = errorCount;
    canMessages[canMessageCount - 1].data[byteNum++] = mcm_getStartupStage(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = mcm_getStartupStage(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = mcm_getStartupStage(mcm);
    canMessages[canMessageCount - 1].data[byteNum++] = mcm_getStartupStage(mcm);

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_allSafe(sc) == TRUE ? 0xFF : 0;
    canMessages[canMessageCount - 1].data[byteNum++] = SafetyChecker_getErrorByte(sc, 0);
    canMessages[canMessageCount - 1].data[byteNum++] = 2;
    canMessages[canMessageCount - 1].data[byteNum++] = 3;
    canMessages[canMessageCount - 1].data[byteNum++] = 4;
    canMessages[canMessageCount - 1].data[byteNum++] = 5;
    canMessages[canMessageCount - 1].data[byteNum++] = 6;
    canMessages[canMessageCount - 1].data[byteNum++] = 7;
    */

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = throttlePercent;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = tps0Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_value;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps0_calibMax >> 8;

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = tps1Percent;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_value;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = tps->tps1_calibMax >> 8;

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = brakePercent; //This should be bps0Percent, but for now bps0Percent = brakePercent
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_value;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_value >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMin;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMin >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMax;
    canMessages[canMessageCount - 1].data[byteNum++] = bps->bps0_calibMax >> 8;

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = brakePercent; //This should be bps0Percent, but for now bps0Percent = brakePercent
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;
    canMessages[canMessageCount - 1].data[byteNum++] = 0;

    canMessageCount++;
    byteNum = 0;
    canMessages[canMessageCount - 1].length = 8; // how many bytes in the message
    canMessages[canMessageCount - 1].id_format = IO_CAN_STD_FRAME;
    canMessages[canMessageCount - 1].id = canMessageID + canMessageCount - 1;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FL) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FL) + 0.5) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FR) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, FR) + 0.5) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RL) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RL) + 0.5) >> 8;
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RR) + 0.5);
    canMessages[canMessageCount - 1].data[byteNum++] = (ubyte2)(WheelSpeeds_getWheelSpeed(wss, RR) + 0.5) >> 8;


    //----------------------------------------------------------------------------
    //Additional sensors
    //----------------------------------------------------------------------------
    //12v battery ---------------------------------------------------    
    /*
    messageIndex++;
    canMessages[messageIndex].id = canMessageBaseId_VCU + 0xA;  //0x50A
    canMessages[messageIndex].id_format = IO_CAN_STD_FRAME;
    canMessages[messageIndex].length = 2;
    canMessages[messageIndex].data[0] = (ubyte1)Sensor_LVBattery.sensorValue;
    canMessages[messageIndex].data[1] = Sensor_LVBattery.sensorValue >> 8;
    */

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    //IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    CanManager_send(me, CAN0_HIPRI, canMessages, canMessageCount);  //Important: Only transmit one message (the MCU message)
    //IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, canMessageCount);  

}