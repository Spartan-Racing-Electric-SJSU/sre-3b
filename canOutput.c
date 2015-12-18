/*****************************************************************************
* CAN Output
******************************************************************************
* Description:
* This file is where we will (1) assemble and (2) send all of the can messaages
* that are sent over CAN.  This includes sensor messages, MCU control, etc.
*
* Assumptions:
* All devices on our car (sensors, controllers... even the VCU itself) are
* represented as objects.  These objects should be up-to-date before these
* functions are called, otherwise CAN messages will be produced with data that
* is outdated (same message as last time).
*
* Usage:
* Each function should perform the following steps->
* 1. Utilize the canMessages[] object (an array of CAN frames), defined in can.h
* 2. Define each CAN message's properties:
*    - id
*    - id_format (always IO_CAN_STD_FRAME - we don't need extended frames)
*    - length
*    - data[]
* 3. Pass the canMessages into the CAN FIFO queue
* Important - Ensure these steps are completed before calling another function
* (e.g. MCU heartbeat message) that also uses same canSensorMessge[] object.
*
******************************************************************************
* To-do:
* Decide whether to use different canMessage[] objs for each function
* , or perhaps just for separate read/write queues
* , or not at all (one object for 2 queues [read/write] * 2 can channels [can0/1])
******************************************************************************
* Revision history:
* 2015-12-07 - Rusty Pedrosa - File created
*****************************************************************************/

#include "IO_CAN.h"
#include "IO_RTC.h"

#include "canOutput.h"
#include "sensors.h"
#include "motorController.h"
#include "can.h"

extern MotorController MCU0;
//extern const ubyte1 canMessageLimit = 10;
extern IO_CAN_DATA_FRAME canMessages[];
//extern const ubyte2 canMessageBaseId_VCU = 0x500;

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
//extern Sensor Sensor_BPS1;
extern Sensor Sensor_WSS_FL;
extern Sensor Sensor_WSS_FR;
extern Sensor Sensor_WSS_RL;
extern Sensor Sensor_WSS_RR;
extern Sensor Sensor_WPS_FL;
extern Sensor Sensor_WPS_FR;
extern Sensor Sensor_WPS_RL;
extern Sensor Sensor_WPS_RR;
extern Sensor Sensor_SPS;
extern Sensor Sensor_LVBattery;

/*****************************************************************************
* Standalone Sensor messages
******************************************************************************
* Load sensor values into CAN messages
* Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
* The message addresses are at:
* https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
****************************************************************************/
void canOutput_sendSensorMessages(void)
{
    
    //Keep track of the current message # that we're working on.
    //(# of messages in the array going to FIFO queue, NOT the same as message ID/address)
    ubyte1 messageIndex = 0;


    //----------------------------------------------------------------------------
    //Standalone sensors
    //----------------------------------------------------------------------------
    //This loop only handles the standalone sensors (CAN address 0x500 to 0x503 [for now])
    for (messageIndex = 0; messageIndex <= 3; messageIndex++)
    {
        canMessages[messageIndex].id = canMessageBaseId_VCU + messageIndex;
        canMessages[messageIndex].id_format = IO_CAN_STD_FRAME;

        switch (messageIndex)
        {
        case 0: //TPS ---------------------------------------------------
            canMessages[messageIndex].length = 8;
            canMessages[messageIndex].data[0] = (ubyte1)Sensor_TPS0.sensorValue;      //TPS0.lowbyte
            canMessages[messageIndex].data[1] = Sensor_TPS0.sensorValue >> 8; //TPS0.hibyte

            canMessages[messageIndex].data[4] = (ubyte1)Sensor_TPS1.sensorValue;      //TPS0.lowbyte
            canMessages[messageIndex].data[5] = Sensor_TPS1.sensorValue >> 8; //TPS0.hibyte

            break;

        case 1: //BPS ---------------------------------------------------
            canMessages[messageIndex].length = 2;
            canMessages[messageIndex].data[0] = (ubyte1)Sensor_BPS0.sensorValue;
            canMessages[messageIndex].data[1] = Sensor_BPS0.sensorValue >> 8;

            break;


        case 2: //WSS ---------------------------------------------------
            canMessages[messageIndex].length = 8;
            canMessages[messageIndex].data[0] = (ubyte1)Sensor_WSS_FL.sensorValue;
            canMessages[messageIndex].data[1] = Sensor_WSS_FL.sensorValue >> 8;
            canMessages[messageIndex].data[2] = (ubyte1)Sensor_WSS_FR.sensorValue;
            canMessages[messageIndex].data[3] = Sensor_WSS_FR.sensorValue >> 8;
            canMessages[messageIndex].data[4] = (ubyte1)Sensor_WSS_RL.sensorValue;
            canMessages[messageIndex].data[5] = Sensor_WSS_RL.sensorValue >> 8;
            canMessages[messageIndex].data[6] = (ubyte1)Sensor_WSS_RR.sensorValue;
            canMessages[messageIndex].data[7] = Sensor_WSS_RR.sensorValue >> 8;
            //TODO: Figure out if cast to ubyte1 is needed

            break;

        case 3: //WPS ---------------------------------------------------
            canMessages[messageIndex].length = 8;
            canMessages[messageIndex].data[0] = (ubyte1)Sensor_WPS_FL.sensorValue;
            canMessages[messageIndex].data[1] = Sensor_WPS_FL.sensorValue >> 8;
            canMessages[messageIndex].data[2] = (ubyte1)Sensor_WPS_FR.sensorValue;
            canMessages[messageIndex].data[3] = Sensor_WPS_FR.sensorValue >> 8;
            canMessages[messageIndex].data[4] = (ubyte1)Sensor_WPS_RL.sensorValue;
            canMessages[messageIndex].data[5] = Sensor_WPS_RL.sensorValue >> 8;
            canMessages[messageIndex].data[6] = (ubyte1)Sensor_WPS_RR.sensorValue;
            canMessages[messageIndex].data[7] = Sensor_WPS_RR.sensorValue >> 8;

            break;

        case 4: //SPS ---------------------------------------------------
            break;

        } //end switch
    } //end for

    //----------------------------------------------------------------------------
    //Additional sensors
    //----------------------------------------------------------------------------
    //12v battery ---------------------------------------------------    
    messageIndex++;
    canMessages[messageIndex].id = canMessageBaseId_VCU + 0xA;  //0x50A
    canMessages[messageIndex].id_format = IO_CAN_STD_FRAME;
    canMessages[messageIndex].length = 2;
    canMessages[messageIndex].data[0] = (ubyte1)Sensor_LVBattery.sensorValue;
    canMessages[messageIndex].data[1] = Sensor_LVBattery.sensorValue >> 8;



    //----------------------------------------------------------------------------
    // CAN
    //----------------------------------------------------------------------------
    //TODO: Make sure it's okay for ConfigFIFO(,,size) to be different from the array's size.


    //Place message into CAN 1 output FIFO queue
    IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, messageIndex + 1);

    //TEMP: Send to both CAN channels
    IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, messageIndex + 1);

    //Place message into CAN 1 output FIFO queue
    //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, &canMessages);
    //IO_CAN_WriteMsg(canFifoHandle_HiPri_Write, &canMessages);
}


/*****************************************************************************
* Motor Controller (MCU) control message
******************************************************************************
* TODO: Parameterize the motor controller (allow multiple motor controllers)
****************************************************************************/
void canOutput_sendMCUControl(bool sendEvenIfNoChanges)
{
    //Only send a message if there's an update or it's been > .25 seconds or force=true
    if ((sendEvenIfNoChanges == TRUE) || (MCU0.commands.updateCount > 1) || (IO_RTC_GetTimeUS(MCU0.commands.timeStamp_lastCommandSent) > 250000))
    {
        //Rinehart CAN control message (heartbeat) structure ----------------
        canMessages[0].length = 8; // how many bytes in the message
        canMessages[0].id_format = IO_CAN_STD_FRAME;
        canMessages[0].id = 0xC0;

        //Torque (Nm * 10)
        ubyte2 mcuTorque = 5; //In Nm * 10. 125 continuous, 240 max
        canMessages[0].data[0] = (ubyte1)MCU0.commands.requestedTorque;
        canMessages[0].data[1] = MCU0.commands.requestedTorque >> 8;

        //Speed (RPM?) - not needed - mcu should be in torque mode
        canMessages[0].data[2] = 0;
        canMessages[0].data[3] = 0;

        //Direction: 0=CW, 1=CCW
        canMessages[0].data[4] = 1; //MCU0.direction;

        //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
        canMessages[0].data[5] = 0; //First set whole byte to zero
    
        //Next add each bit one at a time, starting with the bit that belongs in the leftmost position
        for (int bit = 7; bit >= 0; bit--)
        {
            canMessages[0].data[5] <<= 1;  //Always leftshift first
            switch (bit)
            {
                // Then add your bit to the right (note: the order of case statements doesn't matter - it's the fact that bit-- instead of bit++;)
                case 1: canMessages[0].data[5] |= (MCU0.commands.enableDischarge == TRUE) ? 1 : 0; break; 
                case 0: canMessages[0].data[5] |= (MCU0.commands.enableInverter == TRUE) ? 1 : 0; break;  // Then add your bit to the right

            }
        }

        //Unused (future use)
        canMessages[0].data[6] = 0;
        canMessages[0].data[7] = 0;

        //Place the can messsages into the FIFO queue ---------------------------------------------------
        IO_CAN_WriteFIFO(canFifoHandle_HiPri_Write, canMessages, 1);  //Important: Only transmit one message (the MCU message)
        IO_CAN_WriteFIFO(canFifoHandle_LoPri_Write, canMessages, 1);  //Important: Only transmit one message (the MCU message)

        //Reset the last message count/timestamp
        IO_RTC_StartTime(&MCU0.commands.timeStamp_lastCommandSent);
        MCU0.commands.updateCount = 0;

        //IO_CAN_WriteMsg(canFifoHandle_HiPri_Write, &canMessages);  //Important: Only transmit one message (the MCU message)
        //IO_CAN_WriteMsg(canFifoHandle_LoPri_Write, &canMessages);  //Important: Only transmit one message (the MCU message)


    } //end if sendEvenIfNoChanges/etc
}

