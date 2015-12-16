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
            //canInput_parseMCUMessage();
            switch (canMessages[currMessage].data[6])
            {
                //Motor can only be run if inverterenabled = true and lockoutdisabled=true
                //HUUUUUUUUUUUUGE TODO: Switch this to bit masking or something
            case 0: // = 0b00000000 = 0x00
                MCU0.inverterEnabled = FALSE;
                MCU0.lockoutDisabled = TRUE;
                break;
            case 1: // = 0b00000001 = 0x01
                MCU0.inverterEnabled = TRUE;
                MCU0.lockoutDisabled = TRUE;
                break;
            case 128: // = 0b10000000 = 0x80
                MCU0.inverterEnabled = FALSE;
                MCU0.lockoutDisabled = FALSE;
                break;
            case 129: // = 0b10000000 = 0x81
                MCU0.inverterEnabled = TRUE;
                MCU0.lockoutDisabled = FALSE;
                break;

            //default:
                
            }
            
            break;

        case 0xAB:  //MCU Fault codes

            break;

        //default:

        }
    }

    //Echo message on lopri channel
    IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, messagesReceived);
    //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, canMessages);
}
