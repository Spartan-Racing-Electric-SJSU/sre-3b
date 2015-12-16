#include "IO_Driver.h"
#include "IO_CAN.h"

#include "can.h"
#include "canInput.h"
#include "motorController.h"

extern MotorController MCU0;

//extern const ubyte1 canMessageLimit = 10;
extern IO_CAN_DATA_FRAME canMessages[];

void canInput_readMessages()
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
        //-------------------------------------------------------------------------
        case 0xAA:  
            MCU0.lockoutStatus = (canMessages[currMessage].data[6] & 0x80 == 0x80) ? DISABLED : ENABLED;
            MCU0.inverterStatus = (canMessages[currMessage].data[6] & 1 == 1) ? ENABLED : DISABLED;    
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

        //default:

        }
    }

    //Echo message on lopri channel
    IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, messagesReceived);
    //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
}
