/*****************************************************************************
* VCU Sensor Input Logic
******************************************************************************
* This file contains rough pseudocode for the INPUT functions that will be 
* performed by the VCU.  The objective of writing this code is to gain an 
* understanding for some of the logic that the VCU will perform while 
* avoiding the specific programming complexities of the VCU itself.
* 
* While this file will contain actual calculations used by the VCU, which can
* be complex or confusing, the code itself should be clear enough that anyone
* on the team can follow its logic regardless of their level of understanding
* of computer programming.
*
* Note that only input functions (reading values from sensors) should be
* contained within this file.  Translations into usable units (e.g. voltage
* to throttle percent, for example) DO belong here.
*
* Revision history:
* 2015-11-05 - Rusty Pedrosa - 
* 2015-11-04 - Rusty Pedrosa - Bugfixes, migrated test code into this file
* 2015-10-28 - Rusty Pedrosa - Moved from pseudocode to actual VCU code
* 2015-10-21 - Rusty Pedrosa - Pseudocode complete
* 2015-09-29 - Rusty Pedrosa - Updated file description (this section)
*                            - Colored comments in green
*                            - Clarified some comments about rules
*                            - Added TPSErrorState flag
*                            - Check TPSErrorState before returning pedal %
*                            - Use TPSErrorState in rest of GetTPSValue
*                            - Added pedal % calculations w/calibration
*                            - TODO: Publish calibration excel sheet to group
* 2015-09-28 - Rusty Pedrosa - Created this file.  Working on
*                              calculations for throttle position and calibration.
*****************************************************************************/

//----------------------------------------------------------------------------
// VCU Initialization Stuff
//----------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <math.h>        //Using favg() (average of float datatype)
#include "IO_Driver.h"
#include "APDB.h"
#include "IO_ADC.h"
#include "IO_RTC.h"
#include "IO_CAN.h"

/* Application Database,
 * needed for TTC-Downloader
 */
APDB appl_db =
{     0                      /* ubyte4 versionAPDB        */
    , {0}                    /* BL_T_DATE flashDate       */
                             /* BL_T_DATE buildDate       */
    , {(ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
    , 0                      /* ubyte4 nodeType           */
    , 0                      /* ubyte4 startAddress       */
    , 0                      /* ubyte4 codeSize           */
    , 0                      /* ubyte4 legacyAppCRC       */
    , 0                      /* ubyte4 appCRC             */
    , 1                      /* ubyte1 nodeNr             */
    , 0                      /* ubyte4 CRCInit            */
    , 0                      /* ubyte4 flags              */
    , 0                      /* ubyte4 hook1              */
    , 0                      /* ubyte4 hook2              */
    , 0                      /* ubyte4 hook3              */
    , APPL_START             /* ubyte4 mainAddress        */
    , {0, 1}                 /* BL_T_CAN_ID canDownloadID */
    , {0, 2}                 /* BL_T_CAN_ID canUploadID   */
    , 0                      /* ubyte4 legacyHeaderCRC    */
    , 0                      /* ubyte4 version            */
    , 500                    /* ubyte2 canBaudrate        */
    , 0                      /* ubyte1 canChannel         */
    , {0}                    /* ubyte1 reserved[8*4]      */
    , 0                      /* ubyte4 headerCRC          */
};

ubyte4 timestamp_sensorpoll = 0;

//----------------------------------------------------------------------------
// Sensor Object Definitions
//----------------------------------------------------------------------------
//TODO: Implement a parent Sensor class?
typedef struct _Sensor {
    //CAN stuff
    ubyte1 canMessageID;
    //ubyte1 adc_channel; //Pick one from IO_Constants.h
    
    //Sensor values / properties
    ubyte2 specMin;
    ubyte2 specMax;
    ubyte2 calibMin;
    ubyte2 calibMax;
    bool isCalibrated;
    ubyte2 sensorValue;
    bool fresh;
    ubyte2 calibratedPosition;
    
} Sensor;

//----------------------------------------------------------------------------
// Sensor Object Instantiations
//----------------------------------------------------------------------------
// Write soem nice comments here
//
// The first parameter, Can ID, is REQUIRED for sending data out over CAN.
//
// specMin/Max values should come from each sensor's datasheets, but it is not
// required for all sensors.
//----------------------------------------------------------------------------

//TODO: Read stored calibration data from EEPROM
//Torque Encoders (Throttle)
//Create objects to represent our two TPS's 
//NOTE: Should default values be stored in EEPROM or where ever configuration data goes?
Sensor Sensor_TPS0 = { 0, 0.5, 4.5, 4.5, 0.5, FALSE, 0, FALSE, 0 };
Sensor Sensor_TPS1 = { 0, 4.5, 0.5, 0.5, 4.5, FALSE, 0, FALSE, 0 };

//Brake Position Sensors
//Sensor_BPS.SpecMinV = 0.5;
//Sensor_BPS.SpecMaxV = 4.5;
//Sensor_BPS.SpecMinPSI = 0;     //Is this true? Not specified in datasheet
//Sensor_BPS.SpecMaxPSI = 2000;  //NOTE: % of PSI <> % of Pedal Travel.
Sensor Sensor_BPS0 = { 1, 0.5, 4.5, 4.5, 0.5, FALSE, 0, FALSE, 0 };  //Front/whole system
//Sensor Sensor_BPS1 = { 2, 0.5, 4.5, 4.5, 0.5, FALSE, 0, FALSE, 0 }; //Rear Wheel Speed Sensors
Sensor Sensor_WSS_FL = { 3 };
Sensor Sensor_WSS_FR = { 3 };
Sensor Sensor_WSS_RL = { 3 };
Sensor Sensor_WSS_RR = { 3 };

//Wheel Position Sensors (Shock pots)
Sensor Sensor_WPS_FL = { 4 };
Sensor Sensor_WPS_FR = { 4 };
Sensor Sensor_WPS_RL = { 4 };
Sensor Sensor_WPS_RR = { 4 };

//Switches
//precharge failure

//Other
Sensor Sensor_LVBattery = { 5 };  //Note: no init for this "sensor"

/*****************************************************************************
* Main!!!!!!!!!!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    //----------------------------------------------------------------------------
    // This is where we turn on the VCU's internal power supplies and sensors.
    //
    // The supplies/sensors and their parameters are defined in IO_ADC.h.
    // Each sensor/ADC channel/etc has different parameters, so make sure to
    // check the .h files, the examples, and the documentation!
    //----------------------------------------------------------------------------

    //What does the VCU IO Driver do?  Put comments here.
    IO_Driver_Init(NULL);

    //Create CAN Message Objects ---------------------------------------------------
    //List all CAN IDs here:
    //https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit

    ubyte1 canMessageCount = 4;
    IO_CAN_DATA_FRAME canMessage[canMessageCount];// = { { { 0 } } };  //20 different things being reported over CAN
                                                                    //0 = TPS0,TPS1
                                                                    //1 = BPS0,1
                                                                    //2 = WSS FL,FR,RL,RR
                                                                    //3 = WPS FL,FR,RL,RR

    //Power supplies ---------------------------------------------------
    //Analog sensor supplies
    IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);

    //Variable power supply (used by BPS)
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);    //IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

    //ADC channels ---------------------------------------------------
    //TPS
    IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);

    //BPS
    IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

    //Unused
    //IO_ADC_Channerive RTD device 

    //Shockpots
    IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_05, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_06, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_07, IO_ADC_RESISTIVE, 0, 0, 0, NULL);

    //PWD channels ---------------------------------------------------
    //Wheel Speed Sensors (Why is this called PWD?
    IO_PWD_FreqInit(IO_PWD_08, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?
    IO_PWD_FreqInit(IO_PWD_09, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?
    IO_PWD_FreqInit(IO_PWD_10, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?
    IO_PWD_FreqInit(IO_PWD_11, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?

    for (ubyte1 i = 0; i < canMessageCount; i++)
    {
        canMessage[i].id = i;  //TPS0
        canMessage[i].id_format = IO_CAN_STD_FRAME;
        //canMessage[i].length = 4;  //TPS0 = byte 0,1; TPS1 = byte 1,2; 
    }

    //Activate the CAN channels ---------------------------------------------------
    ubyte1 handle_fifo_w; //What is this for?
    IO_CAN_Init(IO_CAN_CHANNEL_0, 250, 0, 0, 0);
    //IO_CAN_ConfigMsg(&handle_w, IO_CAN_CHANNEL_0, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);
    IO_CAN_ConfigFIFO(&handle_fifo_w
                     , IO_CAN_CHANNEL_0 // channel 0
                     , canMessageCount  // 20 items
                     , IO_CAN_MSG_WRITE // transmit fifo buffer
                     , IO_CAN_STD_FRAME // standard ID
                     , 0
                     , 0);

    //----------------------------------------------------------------------------
    // Initial Power-up functions
    //----------------------------------------------------------------------------
    //TODO: Read calibration data from EEPROM?
    //TODO: Run calibration functions?
    //TODO: Power-on error checking?


    //----------------------------------------------------------------------------
    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a
    * defined cycle time (here: 5 ms)
    */
    //----------------------------------------------------------------------------

    while (1)
    {
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_sensorpoll);

        /* Task begin function for IO Driver
         * This function needs to be called at
         * the beginning of every SW cycle
         */
        IO_Driver_TaskBegin();


        /*
        *  Application Code
        */
        

        /* Add your application code here               */
        /*  - IO driver task functions can be used,     */
        /*    to read/write from/to interfaces and IOs  */

        //Read TPS voltages ---------------------------------------------------
        IO_ADC_Get(IO_ADC_5V_00, &Sensor_TPS0.sensorValue, &Sensor_TPS0.fresh);
        IO_ADC_Get(IO_ADC_5V_01, &Sensor_TPS1.sensorValue, &Sensor_TPS1.fresh);

        //Build TPS CAN message ---------------------------------------------------
        //Each .data[] holds 1 byte - sensor data must be broken up into separate bytes
        canMessage[Sensor_TPS0.canMessageID].length = 4;                                             //Little endian
        canMessage[Sensor_TPS0.canMessageID].data[0] = *((ubyte1*)&Sensor_TPS0.sensorValue);         //TPS0.lowbyte
        canMessage[Sensor_TPS0.canMessageID].data[1] = *((ubyte1*)((&Sensor_TPS0.sensorValue) + 1)); //TPS0.hibyte
        canMessage[Sensor_TPS0.canMessageID].data[2] = *((ubyte1*)&Sensor_TPS1.sensorValue);         //TPS0.lowbyte
        canMessage[Sensor_TPS0.canMessageID].data[3] = *((ubyte1*)((&Sensor_TPS1.sensorValue) + 1)); //TPS0.hibyte

        //Get BPS data from ADC ---------------------------------------------------
        IO_ADC_Get(IO_ADC_5V_02, &Sensor_BPS0.sensorValue, &Sensor_BPS0.fresh);
        //Load data into CAN ---------------------------------------------------
        canMessage[Sensor_BPS0.canMessageID].length = 2;
        canMessage[Sensor_BPS0.canMessageID].data[0] = *((ubyte1*)&Sensor_BPS0.sensorValue);
        canMessage[Sensor_BPS0.canMessageID].data[1] = *((ubyte1*)((&Sensor_BPS0.sensorValue) + 1));

        //?? - For future use ---------------------------------------------------
        //IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

        //Shock pots ---------------------------------------------------
        IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FL.sensorValue, &Sensor_WPS_FL.fresh);
        IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FR.sensorValue, &Sensor_WPS_FR.fresh);
        IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RL.fresh);
        IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RR.fresh);

        canMessage[Sensor_WPS_FL.canMessageID].length = 8;
        canMessage[Sensor_WPS_FL.canMessageID].data[0] = *((ubyte1*)&Sensor_WPS_FL.sensorValue);
        canMessage[Sensor_WPS_FL.canMessageID].data[1] = *((ubyte1*)((&Sensor_WPS_FL.sensorValue) + 1));
        canMessage[Sensor_WPS_FL.canMessageID].data[2] = *((ubyte1*)&Sensor_WPS_FR.sensorValue);        
        canMessage[Sensor_WPS_FL.canMessageID].data[3] = *((ubyte1*)((&Sensor_WPS_FR.sensorValue) + 1)); 
        canMessage[Sensor_WPS_FL.canMessageID].data[4] = *((ubyte1*)&Sensor_WPS_RL.sensorValue);         
        canMessage[Sensor_WPS_FL.canMessageID].data[5] = *((ubyte1*)((&Sensor_WPS_RL.sensorValue) + 1)); 
        canMessage[Sensor_WPS_FL.canMessageID].data[6] = *((ubyte1*)&Sensor_WPS_RR.sensorValue);         
        canMessage[Sensor_WPS_FL.canMessageID].data[7] = *((ubyte1*)((&Sensor_WPS_RR.sensorValue) + 1)); 


        //Wheel speed sensors ---------------------------------------------------
        IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FL.sensorValue);
        IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_FR.sensorValue);
        IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_RL.sensorValue);
        IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RR.sensorValue);

        canMessage[Sensor_WSS_FL.canMessageID].length = 8;
        canMessage[Sensor_WSS_FL.canMessageID].data[0] = *((ubyte1*)&Sensor_WSS_FL.sensorValue);
        canMessage[Sensor_WSS_FL.canMessageID].data[1] = *((ubyte1*)((&Sensor_WSS_FL.sensorValue) + 1));
        canMessage[Sensor_WSS_FL.canMessageID].data[2] = *((ubyte1*)&Sensor_WSS_FR.sensorValue);
        canMessage[Sensor_WSS_FL.canMessageID].data[3] = *((ubyte1*)((&Sensor_WSS_FR.sensorValue) + 1));
        canMessage[Sensor_WSS_FL.canMessageID].data[4] = *((ubyte1*)&Sensor_WSS_RL.sensorValue);
        canMessage[Sensor_WSS_FL.canMessageID].data[5] = *((ubyte1*)((&Sensor_WSS_RL.sensorValue) + 1));
        canMessage[Sensor_WSS_FL.canMessageID].data[6] = *((ubyte1*)&Sensor_WSS_RR.sensorValue);
        canMessage[Sensor_WSS_FL.canMessageID].data[7] = *((ubyte1*)((&Sensor_WSS_RR.sensorValue) + 1));

        //Other stuff
        //Battery voltage (at VCU internal electronics supply input)
        IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);

        canMessage[Sensor_LVBattery.canMessageID].length = 2;
        canMessage[Sensor_LVBattery.canMessageID].data[0] = *((ubyte1*)&Sensor_LVBattery.sensorValue);
        canMessage[Sensor_LVBattery.canMessageID].data[1] = *((ubyte1*)((&Sensor_LVBattery.sensorValue) + 1));

        /*
        //Send can message
        if (IO_CAN_MsgStatus(handle_w) == IO_E_OK)
        {
            IO_CAN_WriteMsg(handle_w, &can_frame[0])
        }
        */
        IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);
        IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);
        IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);
        IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);
        IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);

        /* Task end function for IO Driver
        * This function needs to be called at
        * the end of every SW cycle
        */
        IO_Driver_TaskEnd();

        /* wait until the cycle time is over */
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 5000);   // wait until 5ms have passed
    } //end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    //IO_ADC_ChannelDeInit(IO_ADC_5V_00);




}

