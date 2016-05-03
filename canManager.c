#include "IO_Driver.h" 
#include "IO_CAN.h"
#include "IO_RTC.h"

#include "canManager.h"
#include "avlTree.h"
#include "motorController.h"
#include "bms.h"

struct _CanManager {
    avlNode* incomingTree;
    avlNode* outgoingTree;

    ubyte1 canMessageLimit;
    IO_CAN_DATA_FRAME canMessages[];
	
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
    
    IO_ErrorType ioErr_can0_Init = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can1_Init = IO_E_CAN_BUS_OFF;

    IO_ErrorType ioErr_can0_fifoInit_R = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can0_fifoInit_W = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can1_fifoInit_R = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can1_fifoInit_W = IO_E_CAN_BUS_OFF;

    IO_ErrorType ioErr_can0_read = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can0_write = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can1_read = IO_E_CAN_BUS_OFF;
    IO_ErrorType ioErr_can1_write = IO_E_CAN_BUS_OFF;

    ubyte2 sendDelayMs;
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
	me->rootNode = NULL;
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

    //-------------------------------------------------------------------
    //Define default messages
    //-------------------------------------------------------------------
    //Incoming
    AVL_insert(me->outgoingTree, 0x0C0, 10, 125, TRUE); //MCM command message

    //Outgoing
    AVL_insert(me->incomingTree, 0X0AA, 0, 10000, TRUE); //MCM Status
    AVL_insert(me->incomingTree, 0X0AB, 0, 10000, TRUE); //MCM Faults
    AVL_insert(me->incomingTree, 0x623, 0, 5000, TRUE); //BMS faults
    AVL_insert(me->incomingTree, 0x629, 0, 1000, TRUE); //BMS high priority message

	return me;
}

IO_ErrorType CanManager_send(CanManager* me, CanChannel channel, IO_CAN_DATA_FRAME* canMessages[], ubyte2 canMessageCount, bool force)
{
    bool sendMessage = FALSE;
    ubyte1 messagesToSendCount = 0;
    IO_CAN_DATA_FRAME messagesToSend[canMessageCount] = 0;  //Array will be too big, but oh well
    //(channel == CAN0_HIPRI) ? can
    //Check if message exists in outgoing tree
    AVLNode* lastMessage;
    ubyte1 messagePosition; //used twice
    for (messagePosition = 0, messagePosition < canMessageCount; i++)
    {
        lastMessage = AVL_find(canMessages[messagePosition]->id, me->outgoingTree);

        //This message has never been sent before
        if (lastMessage == NULL)
        {
            lastMessage = AVL_insert(me->outgoingTree, canMessages[messagePosition]->id, canMessages[messagePosition]->data, me->sendDelayMs, me->sendDelayMs, FALSE);
            //send message later
            sendMessage = TRUE;
        }
        else //a message was found
        {
            bool dataChanged == FALSE;
            //Check if data has changed
            for (ubyte1 dataPosition = 0; dataPosition < 8; i++)
            {
                //if any data byte is changed, then probably want to send the message
                if (lastMessage->data[dataPosition] <> canMessages[messagePosition])
                {
                    dataChanged = TRUE;
                }
            }
            
            //If data has changed
            if (dataChanged == TRUE)
            {
                //Send message if >10ms have passed (or whatever the min time is for this message)
                if (IO_RTC_GetTimeUS(lastMessage->lastSent_timeStamp) >= lastMessage->timeBetweenMessages_Min)
                {
                    sendMessage = TRUE;
                }
            }
            else  //data has NOT changed
            {
                //some messages still need to be sent
                if (IO_RTC_GetTimeUS(lastMessage->lastSent_timeStamp) >= lastMessage->timeBetweenMessages_Max)
                {
                    sendMessage = TRUE;
                }
            }
        } //end check if message should be sent
        if (sendMessage == TRUE)
        {
            //see http://stackoverflow.com/questions/1693853/copying-arrays-of-structs-in-c
            messagesToSend[messagesToSendCount++] = canMessages[messagePosition]; //copy the message that needs to be sent into the outgoing messages 
        }
    } //end of loop for each message in outgoing messages

    if (messagesToSendCount > 0)
    {
        //Now actually send the messages to be sent
        *((channel == CAN0_HIPRI) ? &me->ioErr_can0_write : &me->ioErr_can1_write) = IO_CAN_WriteFIFO((channel == CAN0_HIPRI) ? me->can0_writeHandle : me->can1_writeHandle, canMessages, canMessageCount);

        //And mark the messages as sent
        if ((channel == CAN0_HIPRI ? me->ioErr_can0_write : me->ioErr_can1_write) == IO_E_OK)
        {
            for (messagePosition = 0; messagePosition < messagesToSendCount; messagePosition++)
            {
                //TODO: ??????????????????????????????????
                IO_RTC_GetTimeUS(lastMessage->lastMessage_timeStamp); //flag the message as being sent
            }
        }
    }
}
//Check if message data has changed
//Check if it's time to send message yet
//If so, send message and update tree / linked list
//ToDo? write function to re-send message automatically if timebetweenMessages_Max has elapsed? (only applies to rinehart keepalive message)

/*
//Helper functions
ubyte4 CanManager_timeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage)  //Overflows/resets at 74 min
bool CanManager_enoughTimeSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) // timesincelast > timeBetweenMessages_Min
bool CanManager_dataChangedSinceLastTransmit(IO_CAN_DATA_FRAME* canMessage) //bitwise comparison for all data bytes
*/


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
