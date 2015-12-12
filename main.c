/*****************************************************************************
* VCU Logic
******************************************************************************
* This file temporarily contains all input/output functions that will be 
* performed by the VCU.
*
* Initialization of all I/O pins, sensor object definitions, calculations,
* output functions, and CAN messaging are all performed here.  These should
* be split up (how?) later on.
*
******************************************************************************
* To-do:
* - Get CAN_FIFO working
* - Read calibration data from EEPROM on startup
* - Save calibration data to EEPROM from calibration function
*
******************************************************************************
* Revision history:
* 2015-11-16 - Rusty Pedrosa - Fixed RTDS PWM % calculation typecasting bug
*                            - Changed RTDS test pins around
*                            - RTDS code tested successfully - PWM control works!
*                            - Added CAN message definition for Rinehart
*                            - Added TPS calibration check to GetThrottlePosition
*                            - Fixed some comments
* 2015-11-16 - Rusty Pedrosa - Fixed shock pot ADC channel numbers
*                            - Added IO_DO test code (for driving RTDS):
*                              RTDS will be turned on when WPS_FR (shock pot)
*                              is above 1000 ohms.  Turn knob to test output.
*                            - Updated this file's description (above)
*                            - Added more RTDS test outputs
*                              - On/off at 1000 ohms on pins:
*                                  117, 118, 144
*                              - Variable output from 100-1500 ohms:
*                                  105, 106
*                              - Variable output from 250-1000 ohms:
*                                  103, 104, 115, 116
* 2015-11-15 - Rusty Pedrosa - Disabled CAN_FIFO temporarily to fix compile issues
*                            - Disabled unused sensors temporarily
* 2015-11-07 - Rusty Pedrosa - Fixed data splitting into bytes for CAN messages
*                            - Changed all sensors to a single Sensor struct/datatype
*                            - Defined all external sensors for the car
*                            - Added CAN_FIFO functions
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

//-------------------------------------------------------------------
//VCU Initialization Stuff
//-------------------------------------------------------------------

//VCU/C headers
#include <stdio.h>
#include <string.h>
#include "APDB.h"
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
//#include "IO_CAN.h"
//#include "IO_PWM.h"

//Our code
#include "initializations.h"
#include "sensors.h"
#include "canInput.h"
#include "canOutput.h"
#include "outputCalculations.h"
#include "motorController.h"

//Application Database, needed for TTC-Downloader
APDB appl_db =
    { 0                      /* ubyte4 versionAPDB        */
    ,{ 0 }                    /* BL_T_DATE flashDate       */
                          /* BL_T_DATE buildDate                   */
    ,{ (ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
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
    ,{ 0, 1 }                 /* BL_T_CAN_ID canDownloadID */
    ,{ 0, 2 }                 /* BL_T_CAN_ID canUploadID   */
    , 0                      /* ubyte4 legacyHeaderCRC    */
    , 0                      /* ubyte4 version            */
    , 500                    /* ubyte2 canBaudrate        */
    , 0                      /* ubyte1 canChannel         */
    ,{ 0 }                    /* ubyte1 reserved[8*4]      */
    , 0                      /* ubyte4 headerCRC          */
};


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
extern Sensor Sensor_SAS;

/*****************************************************************************
* Main!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
    /* Initialize the IO driver (without safety functions) */
    //TODO: What does the VCU IO Driver do?
    IO_Driver_Init(NULL);
    vcu_initializeADC();  //Activate the VCU's analog-digital I/O
    vcu_initializeCAN();
    vcu_initializeMCU();


    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a defined cycle time (here: 5 ms) */

	ubyte4 timestamp_sensorpoll = 0;
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_sensorpoll);
        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();



        //----------------------------------------------------------------------------
        // DO STUFF!!!!!!!!!!
        //----------------------------------------------------------------------------
        //Get readings from our sensors and other local devices (buttons, 12v battery, etc)
        sensors_updateSensors();

        //canInput - pull messages from CAN FIFO and update our object representations.
        //Also echo can0 messages to can1 for DAQ.
        canInput_readMessages();


        //calculations - Now that we have local sensor data and external data from CAN, we can
        //do actual processing work, from pedal travel calcs to traction control
        //calculations_calculateStuff();

        motorController_setCommands();

        //Drop the sensor readings into CAN (just raw data, not calculated stuff)
        canOutput_sendMCUControl();
        canOutput_sendSensorMessages();


        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------
        //Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();
        //wait until the cycle time is over
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 10000);   // wait until 5ms (5000us) have passed

    } //end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    //IO_ADC_ChannelDeInit(IO_ADC_5V_00);

}


