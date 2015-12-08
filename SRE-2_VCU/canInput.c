#include "IO_Driver.h"

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


    //Determine message type based on ID
    for (int currMessage = 0; currMessage < messagesReceived; currMessage++)
    { 
        switch (canMessages[currMessage].id)
        {
        case 0xAA:  //MCU Internal states
            //-------------------------------------------------------------------------

            //-------------------------------------------------------------------------
            //canInput_parseMCUMessage();
            switch (canMessages[currMessage].data[6])
            {
                //Motor can only be run if inverterenabled = true and lockoutdisabled=true
                //HUUUUUUUUUUUUGE TODO: Switch this to bit masking or something
            case 0:   //0b00000000 
                MCU0.inverterEnabled = FALSE;
                MCU0.lockoutDisabled = TRUE;
                break;
            case 1:   //0b00000001
                MCU0.inverterEnabled = TRUE;
                MCU0.lockoutDisabled = TRUE;
                break;
            case 128: //0b10000000
                MCU0.inverterEnabled = FALSE;
                MCU0.lockoutDisabled = FALSE;
                break;
            case 129: //0b10000000
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

}
