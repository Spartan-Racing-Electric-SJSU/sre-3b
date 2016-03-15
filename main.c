/*****************************************************************************
* SRE-2 Vehicle Control Firmware for the TTTech HY-TTC 50 Controller (VCU)
******************************************************************************
* For project info and history, see https://github.com/spartanracingelectric/SRE-2
* For software/development questions, email rusty@pedrosatech.com
******************************************************************************
* Files
* The Git repository does not contain the complete firmware for SRE-2.  Modules
* provided by TTTech can be found on the CD that accompanied the VCU. These 
* files can be identified by our naming convetion: TTTech files start with a
* prefix in all caps (such as IO_Driver.h), except for ptypes_xe167.h which
* they also provided.
* For instructions on setting up a build environment, see the SRE-2 getting-
* started document, Programming for the HY-TTC 50, at http://1drv.ms/1NQUppu
******************************************************************************
* Organization
* Our code is laid out in the following manner:
* 
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
//#include "outputCalculations.h"
#include "motorController.h"
#include "readyToDriveSound.h"
#include "torqueEncoder.h"

#include "sensorCalculations.h"

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
 
extern Sensor Sensor_RTDButton;
extern Sensor Sensor_TEMP_BrakingSwitch;
extern Sensor Sensor_EcoButton;

/*****************************************************************************
* Main!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems

    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    vcu_initializeADC();  //Configure and activate all I/O pins on the VCU
    vcu_initializeCAN();
    vcu_initializeSensors();
    //vcu_initializeMCU();

    //Do some loops until the ADC stops outputting garbage values
    vcu_ADCWasteLoop();

    //----------------------------------------------------------------------------
    // External Devices - Object Initializations
    //----------------------------------------------------------------------------
    ReadyToDriveSound* rtds = RTDS_new();
    MotorController* mcm0 = MotorController_new(0xA0, FORWARD, 100);
    TorqueEncoder* tps = TorqueEncoder_new(TRUE);
    //BatteryManagementSystem* bms = BMS_new();

    //----------------------------------------------------------------------------
    // TODO: Additional Initial Power-up functions
    //----------------------------------------------------------------------------
    //TODO: Read calibration data from EEPROM?
    //TODO: Run calibration functions?
    //TODO: Power-on error checking?

    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a defined cycle time (here: 5 ms) */

	ubyte4 timestamp_sensorpoll = 0;
    ubyte1 calibrationErrors;

    //IO_RTC_StartTime(&timestamp_calibStart);
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_sensorpoll);
        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        //Run calibration if commanded
        //if (IO_RTC_GetTimeUS(timestamp_calibStart) < (ubyte4)5000000)
        if (Sensor_EcoButton.sensorValue)
        {
            //calibrateTPS(TRUE, 5);
            TorqueEncoder_startCalibration(tps, 5);
            //DIGITAL OUTPUT 4 for STATUS LED
        }
        TorqueEncoder_calibrationCycle(tps, &calibrationErrors);

        //----------------------------------------------------------------------------
        // Handle data input streams
        //----------------------------------------------------------------------------
        //Get readings from our sensors and other local devices (buttons, 12v battery, etc)
        sensors_updateSensors();

        //canInput - pull messages from CAN FIFO and update our object representations.
        //Also echo can0 messages to can1 for DAQ.
        canInput_readMessages(mcm0);

        //----------------------------------------------------------------------------
        // Calculations
        //----------------------------------------------------------------------------
        //calculations - Now that we have local sensor data and external data from CAN, we can
        //do actual processing work, from pedal travel calcs to traction control
        //calculations_calculateStuff();

        //----------------------------------------------------------------------------
        // Motor Controller Output Calculations
        //----------------------------------------------------------------------------
        //Handle motor startup procedures
        MotorControllerPowerManagement(mcm0, rtds);

        //Assign motor controls to MCM command message
        //motorController_setCommands(rtds);
        //DOES NOT set inverter command or rtds flag
        setMCMCommands(mcm0, rtds);



        //Drop the sensor readings into CAN (just raw data, not calculated stuff)
        canOutput_sendMCUControl(mcm0, FALSE);
        canOutput_sendDebugMessage(tps);
        //canOutput_sendSensorMessages();
        //canOutput_sendStatusMessages(mcm0);


        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------
        RTDS_shutdownHelper(rtds); //Stops the RTDS from playing if the set time has elapsed

        //Task end function for IO Driver - This function needs to be called at the end of every SW cycle
        IO_Driver_TaskEnd();
        //wait until the cycle time is over
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 1000);   // wait until 1ms (1000us) have passed

    } //end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    //IO_ADC_ChannelDeInit(IO_ADC_5V_00);
    //Free memory if object won't be used anymore

}


