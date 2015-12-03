#include "sensors.h"
#include "IO_CAN.h"

#include "canOutput.h"

//----------------------------------------------------------------------------
// Sensor messages
//----------------------------------------------------------------------------
// Load sensor values into CAN messages
// Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
// The message addresses are at:
// https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
//----------------------------------------------------------------------------

void broadcastSensorCanMessages(void)
{
    //Each CAN message needs:
    //id
    //id_format
    //length
    //data[]
    
    //Keep track of the current message # that we're working on.
    //(# of messages in the array going to FIFO queue, NOT the same as message ID/address)
    ubyte1 messageIndex = 0;


    //----------------------------------------------------------------------------
    //Standalone sensors
    //----------------------------------------------------------------------------
    //This loop only handles the standalone sensors (CAN address 0x500 to 0x503 [for now])
    for (messageIndex = 0; messageIndex <= 3; messageIndex++)
    {
        canSensorMessages[messageIndex].id = canMessageIdOffset + messageIndex;
        canSensorMessages[messageIndex].id_format = IO_CAN_STD_FRAME;

        switch (messageIndex)
        {
        case 0: //TPS ---------------------------------------------------
            canSensorMessages[messageIndex].length = 8;
            canSensorMessages[messageIndex].data[0] = Sensor_TPS0.sensorValue;      //TPS0.lowbyte
            canSensorMessages[messageIndex].data[1] = Sensor_TPS0.sensorValue >> 8; //TPS0.hibyte

            canSensorMessages[messageIndex].data[4] = Sensor_TPS1.sensorValue;      //TPS0.lowbyte
            canSensorMessages[messageIndex].data[5] = Sensor_TPS1.sensorValue >> 8; //TPS0.hibyte

            break;

        case 1: //BPS ---------------------------------------------------
            canSensorMessages[messageIndex].length = 2;
            canSensorMessages[messageIndex].data[0] = Sensor_BPS0.sensorValue;
            canSensorMessages[messageIndex].data[1] = Sensor_BPS0.sensorValue >> 8;

            break;


        case 2: //WSS ---------------------------------------------------
            canSensorMessages[messageIndex].length = 8;
            canSensorMessages[messageIndex].data[0] = Sensor_WSS_FL.sensorValue;
            canSensorMessages[messageIndex].data[1] = Sensor_WSS_FL.sensorValue >> 8;
            canSensorMessages[messageIndex].data[2] = Sensor_WSS_FR.sensorValue;
            canSensorMessages[messageIndex].data[3] = Sensor_WSS_FR.sensorValue >> 8;
            canSensorMessages[messageIndex].data[4] = Sensor_WSS_RL.sensorValue;
            canSensorMessages[messageIndex].data[5] = Sensor_WSS_RL.sensorValue >> 8;
            canSensorMessages[messageIndex].data[6] = Sensor_WSS_RR.sensorValue;
            canSensorMessages[messageIndex].data[7] = Sensor_WSS_RR.sensorValue >> 8;
            //TODO: Figure out if cast to ubyte1 is needed

            break;

        case 3: //WPS ---------------------------------------------------
            canSensorMessages[messageIndex].length = 8;
            canSensorMessages[messageIndex].data[0] = (ubyte1)Sensor_WPS_FL.sensorValue;
            canSensorMessages[messageIndex].data[1] = Sensor_WPS_FL.sensorValue >> 8;
            canSensorMessages[messageIndex].data[2] = (ubyte1)Sensor_WPS_FR.sensorValue;
            canSensorMessages[messageIndex].data[3] = Sensor_WPS_FR.sensorValue >> 8;
            canSensorMessages[messageIndex].data[4] = (ubyte1)Sensor_WPS_RL.sensorValue;
            canSensorMessages[messageIndex].data[5] = Sensor_WPS_RL.sensorValue >> 8;
            canSensorMessages[messageIndex].data[6] = (ubyte1)Sensor_WPS_RR.sensorValue;
            canSensorMessages[messageIndex].data[7] = Sensor_WPS_RR.sensorValue >> 8;

            break;
        } //end switch
    } //end for

    //----------------------------------------------------------------------------
    //Additional sensors
    //----------------------------------------------------------------------------
    //12v battery ---------------------------------------------------    
    messageIndex++;
    canSensorMessages[messageIndex].id = canMessageBaseId + 0xA;  //0x50A
    canSensorMessages[messageIndex].id_format = IO_CAN_STD_FRAME;
    canSensorMessages[messageIndex].length = 2;
    canSensorMessages[messageIndex].data[0] = Sensor_LVBattery.sensorValue;
    canSensorMessages[messageIndex].data[1] = Sensor_LVBattery.sensorValue >> 8;



    //----------------------------------------------------------------------------
    // CAN
    //----------------------------------------------------------------------------
    //TODO: Make sure it's okay for ConfigFIFO(,,size) to be different from the array's size.

    //Activate the CAN channels ---------------------------------------------------
    ubyte1 handle_fifo_w; //This is the representation of the CAN FIFO write queue
    IO_CAN_Init(IO_CAN_CHANNEL_0, canSpeed_Channel0, 0, 0, 0);
    IO_CAN_ConfigFIFO(&handle_fifo_w, IO_CAN_CHANNEL_0, messageIndex + 1, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);

    //Place the can messsages into the FIFO queue ---------------------------------------------------
    IO_CAN_WriteFIFO(handle_fifo_w, canSensorMessages, messageIndex + 1);
}