/*****************************************************************************
* Sensors
******************************************************************************
* bla bla bla.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-12-01 - Rusty Pedrosa - Changed loading of sensor data to switch
*                              statement inside of a loop
*****************************************************************************/


#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "sensors.h"


#include "IO_CAN.h"


void updateSensors()
{
    //----------------------------------------------------------------------------
    // Read sensors values from ADC channels
    // The sensor values should be stored in sensor objects.
    //----------------------------------------------------------------------------

    //TODO: Handle errors (using the return values for these Get functions)

    //TODO: RTDS

    //Torque Encoders ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_00, &Sensor_TPS0.sensorValue, &Sensor_TPS0.fresh);
    IO_ADC_Get(IO_ADC_5V_01, &Sensor_TPS1.sensorValue, &Sensor_TPS1.fresh);

    //Brake Position Sensor ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_02, &Sensor_BPS0.sensorValue, &Sensor_BPS0.fresh);

    //?? - For future use ---------------------------------------------------
    //IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

    //Shock pots ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FL.sensorValue, &Sensor_WPS_FL.fresh);
    IO_ADC_Get(IO_ADC_5V_05, &Sensor_WPS_FR.sensorValue, &Sensor_WPS_FR.fresh);
    IO_ADC_Get(IO_ADC_5V_06, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RL.fresh);
    IO_ADC_Get(IO_ADC_5V_07, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RR.fresh);


    //Wheel speed sensors ---------------------------------------------------
    IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FL.sensorValue);
    IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_FR.sensorValue);
    IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_RL.sensorValue);
    IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RR.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);




    //----------------------------------------------------------------------------
    // Load sensor values into CAN messages
    // Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
    //----------------------------------------------------------------------------

    //Create CAN Message Objects ---------------------------------------------------
    const ubyte1 canMessageCount = 1;
    IO_CAN_DATA_FRAME canSensorMessage[canMessageCount] = { { { 0 } } };

    for (ubyte1 i = 0x500; i < canMessageCount; i++)
    {
        canSensorMessage[i].id = canMessageIdOffset + i;
        canSensorMessage[i].id_format = IO_CAN_STD_FRAME;
        
        switch(i)
        {
        case 0: //TPS sensors
            canSensorMessage[i].length = 8;
            canSensorMessage[0].data[0] = Sensor_TPS0.sensorValue;      //TPS0.lowbyte
            canSensorMessage[0].data[1] = Sensor_TPS0.sensorValue >> 8; //TPS0.hibyte

            canSensorMessage[0].data[4] = Sensor_TPS1.sensorValue;      //TPS0.lowbyte
            canSensorMessage[0].data[5] = Sensor_TPS1.sensorValue >> 8; //TPS0.hibyte
            break;

        case 1: //BPS sensor(s)
            canSensorMessage[i].length = 8;

            break;

        case 2:
            canSensorMessage[i].length = 8;

            break;

        case 3:
            canSensorMessage[i].length = 8;

            break;

        case 4:
            canSensorMessage[i].length = 8;

            break;

        }
    }

    //Activate the CAN channels ---------------------------------------------------
    ubyte1 handle_fifo_w; //What is this for?
    IO_CAN_Init(IO_CAN_CHANNEL_0, 250, 0, 0, 0);
    IO_CAN_ConfigFIFO(&handle_fifo_w, IO_CAN_CHANNEL_0, canMessageCount, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);



    //TPS CAN messages (both TPS's in same address
    canSensorMessage[0].id = i;
    canSensorMessage[0].id_format = IO_CAN_STD_FRAME;
    canSensorMessage[0].length = 4;                             //Little endian

    canSensorMessage[Sensor_BPS0.canMessageID].length = 2;
    canSensorMessage[Sensor_BPS0.canMessageID].data[0] = Sensor_BPS0.sensorValue;
    canSensorMessage[Sensor_BPS0.canMessageID].data[1] = Sensor_BPS0.sensorValue >> 8;


    canSensorMessage[Sensor_WPS_FL.canMessageID].length = 8;
    canSensorMessage[Sensor_WPS_FL.canMessageID].data[0] = (ubyte1)Sensor_WPS_FL.sensorValue;
    canSensorMessage[Sensor_WPS_FL.canMessageID].data[1] = Sensor_WPS_FL.sensorValue >> 8;
    canSensorMessage[Sensor_WPS_FR.canMessageID].data[2] = (ubyte1)Sensor_WPS_FR.sensorValue;
    canSensorMessage[Sensor_WPS_FR.canMessageID].data[3] = Sensor_WPS_FR.sensorValue >> 8;
    canSensorMessage[Sensor_WPS_RL.canMessageID].data[4] = (ubyte1)Sensor_WPS_RL.sensorValue;
    canSensorMessage[Sensor_WPS_RL.canMessageID].data[5] = Sensor_WPS_RL.sensorValue >> 8;
    canSensorMessage[Sensor_WPS_RR.canMessageID].data[6] = (ubyte1)Sensor_WPS_RR.sensorValue;
    canSensorMessage[Sensor_WPS_RR.canMessageID].data[7] = Sensor_WPS_RR.sensorValue >> 8;


    canSensorMessage[Sensor_WSS_FL.canMessageID].length = 8;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[0] = Sensor_WSS_FL.sensorValue;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[1] = Sensor_WSS_FL.sensorValue >> 8;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[2] = Sensor_WSS_FR.sensorValue;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[3] = Sensor_WSS_FR.sensorValue >> 8;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[4] = Sensor_WSS_RL.sensorValue;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[5] = Sensor_WSS_RL.sensorValue >> 8;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[6] = Sensor_WSS_RR.sensorValue;
    canSensorMessage[Sensor_WSS_FL.canMessageID].data[7] = Sensor_WSS_RR.sensorValue >> 8;

    canSensorMessage[Sensor_LVBattery.canMessageID].length = 2;
    canSensorMessage[Sensor_LVBattery.canMessageID].data[0] = Sensor_LVBattery.sensorValue;
    canSensorMessage[Sensor_LVBattery.canMessageID].data[1] = Sensor_LVBattery.sensorValue >> 8;


    //----------------------------------------------------------------------------
    // CAN
    //----------------------------------------------------------------------------
    IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);

}