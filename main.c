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
#include "IO_DIO.h"
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
//#include "IO_CAN.h"
//#include "IO_PWM.h"

//Our code
#include "initializations.h"
#include "sensors.h"
#include "canManager.h"
#include "motorController.h"
#include "readyToDriveSound.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "wheelSpeeds.h"
#include "safety.h"
#include "sensorCalculations.h"
#include "serial.h"

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
    ubyte4 timestamp_sensorpoll = 0;
    ubyte4 timestamp_EcoButton = 0;
    ubyte1 calibrationErrors;  //NOT USED

    /*******************************************/
    /*            Initializations              */
    /*******************************************/
    IO_Driver_Init(NULL); //Handles basic startup for all VCU subsystems

    //Special: Initialize serial first so we can use it to debug init of other subsystems
    SerialManager* serialMan = SerialManager_new();
    IO_RTC_StartTime(&timestamp_sensorpoll);
    SerialManager_send(serialMan, "\n\n");
    //SerialManager_send(serialMan, IO_RTC_GetTimeUS(timestamp_sensorpoll));
    SerialManager_send(serialMan, "VCU serial is online.\n");


    //----------------------------------------------------------------------------
    // Check if we're on the bench or not
    //----------------------------------------------------------------------------
    bool bench;
    IO_DI_Init(IO_DI_06, IO_DI_PD_10K);
    IO_RTC_StartTime(&timestamp_sensorpoll);
    while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 999999) //This time doesn't matter
    {
        IO_Driver_TaskBegin();

        //IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
        IO_DI_Get(IO_DI_06, &bench);

        IO_Driver_TaskEnd();
        //TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 10000);   // wait until 10ms have passed
    }
    IO_DI_DeInit(IO_DI_06);
    //Need to invert bench bceause grounded = false
    SerialManager_send(serialMan, bench == TRUE ? "VCU is in bench mode.\n" : "VCU is NOT in bench mode.\n");
    
    //----------------------------------------------------------------------------
    // VCU Subsystem Initializations
    // Eventually, all of these functions should be made obsolete by creating
    // objects instead, like the RTDS/MCM/TPS objects below
    //----------------------------------------------------------------------------
    SerialManager_send(serialMan, "VCU objects/subsystems initializing.\n");
    vcu_initializeADC(bench);  //Configure and activate all I/O pins on the VCU
    //vcu_initializeCAN();
    vcu_initializeSensors(bench);
    //vcu_initializeMCU();

    //Do some loops until the ADC stops outputting garbage values
    vcu_ADCWasteLoop();

    //vcu_init functions may have to be performed BEFORE creating CAN Manager object
    CanManager* canMan = CanManager_new(500, 40, 40, 500, 20, 20, 250000);  //3rd param = messages per node (can0/can1; read/write)


    //----------------------------------------------------------------------------
    // Object representations of external devices
    // Most default values for things should be specified here
    //----------------------------------------------------------------------------    
    ReadyToDriveSound* rtds = RTDS_new();
	//BatteryManagementSystem* bms = BMS_new();
    MotorController* mcm0 = MotorController_new(serialMan, 0xA0, FORWARD, 100); //CAN addr, direction, torque limit x10 (100 = 10Nm)
	TorqueEncoder* tps = TorqueEncoder_new(bench);
	BrakePressureSensor* bps = BrakePressureSensor_new();
	WheelSpeeds* wss = WheelSpeeds_new(18, 18, 16, 16);
	SafetyChecker* sc = SafetyChecker_new(serialMan);
	BatteryManagementSystem* bms = BMS_new(0x620);

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

    //IO_RTC_StartTime(&timestamp_calibStart);
    SerialManager_send(serialMan, "VCU initializations complete.  Entering main loop.\n");
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff (start)
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_sensorpoll);
        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        //SerialManager_send(serialMan, "VCU has entered main loop.");

        /*******************************************/
        /*              Read Inputs                */
        /*******************************************/
        //----------------------------------------------------------------------------
        // Handle data input streams
        //----------------------------------------------------------------------------
        //Get readings from our sensors and other local devices (buttons, 12v battery, etc)
		sensors_updateSensors();

        //canInput - pull messages from CAN FIFO and update our object representations.
        //Also echo can0 messages to can1 for DAQ.
        CanManager_read(canMan, CAN0_HIPRI, mcm0, bms);

        /*******************************************/
        /*          Perform Calculations           */
        /*******************************************/
        //calculations - Now that we have local sensor data and external data from CAN, we can
        //do actual processing work, from pedal travel calcs to traction control
        //calculations_calculateStuff();

        //Run calibration if commanded
		//if (IO_RTC_GetTimeUS(timestamp_calibStart) < (ubyte4)5000000)
		if (Sensor_EcoButton.sensorValue == TRUE)
		{
            if (timestamp_EcoButton == 0)
            {
                SerialManager_send(serialMan, "Eco button detected\n");
                IO_RTC_StartTime(&timestamp_EcoButton);
            }
            else if (IO_RTC_GetTimeUS(timestamp_EcoButton) >= 3000000)
            {
                SerialManager_send(serialMan, "Eco button held 3s - starting calibrations\n");
                //calibrateTPS(TRUE, 5);
                TorqueEncoder_startCalibration(tps, 5);
                BrakePressureSensor_startCalibration(bps, 5);
                //Light_set(Light_dashError, 1);
                //DIGITAL OUTPUT 4 for STATUS LED
            }
		}
        else
        {
            Light_set(Light_dashEco, 0);
            timestamp_EcoButton = 0;
        }
		TorqueEncoder_update(tps);
        //Every cycle: if the calibration was started and hasn't finished, check the values again
        TorqueEncoder_calibrationCycle(tps, &calibrationErrors); //Todo: deal with calibration errors
		BrakePressureSensor_update(bps, bench);
		BrakePressureSensor_calibrationCycle(bps, &calibrationErrors);

		//TractionControl_update(tps, mcm0, wss, daq);

		WheelSpeeds_update(wss);
		//DataAquisition_update(); //includes accelerometer
		//TireModel_update()
		//ControlLaw_update();
		/*
		ControlLaw //Tq command
			TireModel //used by control law -> read from WSS, accelerometer
			StateObserver //choose driver command or ctrl law
		*/	


        //Assign motor controls to MCM command message
        //motorController_setCommands(rtds);
        //DOES NOT set inverter command or rtds flag
        MCM_calculateCommands(mcm0, tps, bps);

        SafetyChecker_update(sc, tps, bps, &Sensor_HVILTerminationSense, &Sensor_LVBattery);

        /*******************************************/
        /*  Output Adjustments by Safety Checker   */
        /*******************************************/
        //SafetyChecker_ReduceTorque()
        //SafetyChecker_?

        /*******************************************/
        /*              Enact Outputs              */
        /*******************************************/
        //MOVE INTO SAFETYCHECKER
        //SafetyChecker_setErrorLight(sc);
        Light_set(Light_dashError, (SafetyChecker_getWarnings(sc) == 0 || SafetyChecker_allSafe(sc) == TRUE) ? 0 : 1);

        //Handle motor controller startup procedures
        MCM_relayControl(mcm0, &Sensor_HVILTerminationSense);
        MotorControllerPowerManagement(mcm0, tps, rtds);

        //Drop the sensor readings into CAN (just raw data, not calculated stuff)
        //MotorController_sendCommandMessage();
        //canOutput_sendMCUControl(mcm0, FALSE);

        //Send debug data
        canOutput_sendDebugMessage(canMan, tps, bps, mcm0, wss, sc);
        //canOutput_sendSensorMessages();
        //canOutput_sendStatusMessages(mcm0);

        //----------------------------------------------------------------------------
        // Task management stuff (end)
        //----------------------------------------------------------------------------
        RTDS_shutdownHelper(rtds); //Stops the RTDS from playing if the set time has elapsed

        IO_UART_Task();  //The task function shall be called every SW cycle.
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


