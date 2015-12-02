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
* TODO:
* - Get CAN_FIFO working
* - Read calibration data from EEPROM on startup
* - Save calibration data to EEPROM from calibration function
*
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
#include <stdio.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_ADC.h"
#include "IO_RTC.h"
#include "APDB.h"
#include "IO_CAN.h"
#include "IO_DIO.h"
#include "IO_PWM.h"
#include <math.h>

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


//----------------------------------------------------------------------------
// Sensor Object Definitions
//----------------------------------------------------------------------------
// Parameters:
// canMessageID - REQUIRED - Unique ID (address) for each CAN message.  Multiple sensors
//                can be assigned to a single ID/message if they'll fit in the
//                available data space
//                TODO: What's the limit?
//				  https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
// TODO: Write the rest of these
//
// specMin/Max values should come from each sensor's datasheets, but it is not
// required for all sensors.
//
// TODO: What about having default calbiration values?  (Probably useless)
//----------------------------------------------------------------------------
typedef struct _Sensor {
    //CAN stuff
    ubyte1 canMessageID;  //Required!
    
    //Sensor values / properties
    ubyte2 specMin;
    ubyte2 specMax;
    bool isCalibrated;
    ubyte2 calibMin;
    ubyte2 calibMax;
	ubyte2 calibNormal;  //zero value or normal position
	
    ubyte2 sensorValue;
    bool fresh;
	
    ubyte2 calibratedValue;
    
} Sensor;


//----------------------------------------------------------------------------
// Sensor Object Instantiations
//----------------------------------------------------------------------------
// canMessageID should be specified NOW!
//----------------------------------------------------------------------------

//TODO: Read stored calibration data from EEPROM

//Torque Encoders (TPS is not really accurate since there's no throttle to position in an EV)
Sensor Sensor_TPS0 = { 0, 0.5, 4.5 };
Sensor Sensor_TPS1 = { 0, 4.5, 0.5 };

//Brake Position Sensors
Sensor Sensor_BPS0 = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
//Sensor Sensor_BPS1 = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)

//Wheel Speed Sensors (like an ABS sensor)
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
Sensor Sensor_LVBattery = { 5 };  //Note: There will be no init for this "sensor"

/*****************************************************************************
* Main!
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

    /* Initialize the IO driver (without safety functions) */
    //TODO: What does the VCU IO Driver do?
    IO_Driver_Init(NULL);
	
	//Power supplies ---------------------------------------------------
    //Analog sensor supplies
    IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);

    //Variable power supply (used by BPS)
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);    //IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

    //Digital outputs ---------------------------------------------------
    //IO_DO_Init(IO_DO_00);

    //Digital PWM outputs ---------------------------------------------------
    //We're not using these
    IO_PWM_Init(IO_PWM_07, 700, TRUE, FALSE, 0, FALSE, NULL);  //Temporary RTDS output

    //ADC channels ---------------------------------------------------
    //TPS
    IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);  
    IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);

    //BPS
    IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

    //Unused
    //IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

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
	
    //----------------------------------------------------------------------------
    // TODO: Initial Power-up functions
    //----------------------------------------------------------------------------
    //TODO: Read calibration data from EEPROM?
    //TODO: Run calibration functions?
    //TODO: Power-on error checking?


    /*******************************************/
    /*       PERIODIC APPLICATION CODE         */
    /*******************************************/
    /* main loop, executed periodically with a
    * defined cycle time (here: 5 ms)
    */

	ubyte4 timestamp_sensorpoll = 0;
    while (1)
    {
        //----------------------------------------------------------------------------
        // Task management stuff
        //----------------------------------------------------------------------------
        //Get a timestamp of when this task started from the Real Time Clock
        IO_RTC_StartTime(&timestamp_sensorpoll);

        //Mark the beginning of a task - what does this actually do?
        IO_Driver_TaskBegin();

        //----------------------------------------------------------------------------
        // Read sensors and insert their data into CAN messages
        // Each can message's .data[] holds 1 byte - sensor data must be broken up into separate bytes
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

        //RTDS tests ---------------------------------------------------
        //Control the RTDS with a pot
        //Pot goes from 2.2 ohm to 4930 ohm
        //Note: There's a problem with the old RTDS where it plays sound
        //      even at 0 duty cycle / DO=FALSE.  Gotta figure out why this
        //      happens and if there's a problem with the VCU.
        
        //Hook up RTDS to pin 103
        float4 dutyPercent;  //Percent (some fraction between 0 and 1)
        ubyte2 dutyHex;

        dutyPercent = getPercent((float4)Sensor_WPS_FR.sensorValue, 500, 2500, TRUE);

        //Set the volume level (0 to 65535.. or 0 to FFFF as seen by VCU)
        dutyHex = 65535 * dutyPercent;
        IO_PWM_SetDuty(IO_PWM_06, 0, NULL);  //Pin 115 - for testing old RTDS
        IO_PWM_SetDuty(IO_PWM_07, dutyHex, NULL);  //Pin 103
        
        //Wheel speed sensors ---------------------------------------------------
        IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FL.sensorValue);
        IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_FR.sensorValue);
        IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_RL.sensorValue);
        IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RR.sensorValue);
        
        //Other stuff ---------------------------------------------------
        //Battery voltage (at VCU internal electronics supply input)
        IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);


        /* Task end function for IO Driver
        * This function needs to be called at
        * the end of every SW cycle
        */		
        IO_Driver_TaskEnd();

        /* wait until the cycle time is over */
        //&& IO_CAN_MsgStatus(handle_fifo_w) != IO_E_OK
        while (IO_RTC_GetTimeUS(timestamp_sensorpoll) < 5000);   // wait until 5ms have passed
    } //end of main loop

    //----------------------------------------------------------------------------
    // VCU Subsystem Deinitializations
    //----------------------------------------------------------------------------
    //IO_ADC_ChannelDeInit(IO_ADC_5V_00);

}


/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
float4 getPercent(float4 value, float4 min, float4 max, bool zeroToOneOnly)
{
    float4 retVal = (value - min) / (max - min);

    if (zeroToOneOnly == TRUE)
    {
        if (retVal > 1) { retVal = 1; }
        else if (retVal < 0) { retVal = 0; }
    }

    return retVal;
}

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/

// Physical pedal travel will only occur across the center (about 1/2) of the actual sensor's range of travel
// The rules (especially EV2.3.6) are written about % of PEDAL travel, not percent of sensor range, so we must calculate pedal travel by recording the min/max voltages at min/max throttle positions

/*-------------------------------------------------------------------
    * CalibrateTPS
    * Description: Records TPS minimum/maximum voltages (when?) and stores them (where?)
    * Parameters:  
    * Inputs:      
    * Returns:     
    * Notes:       
    * Throws:      
    -------------------------------------------------------------------*/
void calibrateTPS(ubyte1 secondsToRunCalibration)
{
    //TODO: Make sure the main loop is running before doing this

    //Reset calibration data
    Sensor_TPS0.calibMin = Sensor_TPS0.specMax;
    Sensor_TPS0.calibMax = Sensor_TPS0.specMin;
    Sensor_TPS0.isCalibrated = FALSE;
    Sensor_TPS1.calibMin = Sensor_TPS1.specMax;
    Sensor_TPS1.calibMax = Sensor_TPS1.specMin;
    Sensor_TPS1.isCalibrated = FALSE;

    ubyte4 timestamp_calibrationstart = 0;
    IO_RTC_StartTime(&timestamp_calibrationstart);  //Get a timestamp of when this task started from the Real Time Clock
    //TODO: Don't start coundown until values have been set
    while (IO_RTC_GetTimeUS(timestamp_calibrationstart) < (ubyte4)secondsToRunCalibration * 1000 * 1000)   // Give the user # seconds to do this
    {
        //TODO: Do something on the display to show that voltages are being recorded
        //Idea: Display "bars" filling up on right segment (for gas pedal) _=E=_=E...
        //      Once calibration data makes sense, show pedal location (0-10%, 10-90%, 90-100%) with bars

        //Record min/max voltages
        if (Sensor_TPS0.sensorValue < Sensor_TPS0.calibMin) {
            Sensor_TPS0.calibMin = Sensor_TPS0.sensorValue;
        }
        if (Sensor_TPS0.sensorValue > Sensor_TPS0.calibMax) {
            Sensor_TPS0.calibMax = Sensor_TPS0.sensorValue;
        }
        if (Sensor_TPS1.sensorValue < Sensor_TPS1.calibMin) {
            Sensor_TPS1.calibMin = Sensor_TPS1.sensorValue;
        }
        if (Sensor_TPS1.sensorValue > Sensor_TPS1.calibMax) {
            Sensor_TPS1.calibMax = Sensor_TPS1.sensorValue;
        }

    }

    //TODO: Write calibration data to EEPROM

    //TODO: Check for valid/reasonable calibration data
    Sensor_TPS0.isCalibrated = TRUE;
    Sensor_TPS1.isCalibrated = TRUE;

    //TODO: Return the display to normal
}   
 
/*-------------------------------------------------------------------
    * GetThrottlePosition
    * Description: Reads TPS Pin voltages and returns % of throttle pedal travel.
    * Parameters:  None
    * Inputs:      Assumes TPS#.sensorValue has been set by main loop
    * Returns:     Throttle value in percent (from 0 to 1)
    * Notes:       Valid pedal travel is from 10% (0.10) to 90% (0.90), not including mechanical limits.
    * Throws:      000 - TPS0 voltage out of range
    *              001 - TPS1 voltage out of range, 002
    -------------------------------------------------------------------*/
float4 GetThrottlePosition(void)
{
    //TODO: Should we calculate this during the main loop? (probably not - what if it's not calibrated?)
    //      or should we only run this when someone specifically asks for throttle position? (probably)

    ubyte1 TPSErrorState = 0; //No errors have been detected so far.

    //-------------------------------------------------------------------
    //First check for implausibility at the pin level
    //USE SPEC SHEET VALUES, NOT CALIBRATION VALUES
    //RULE: EV2.3.10 - signal outside of operating range is considered a failure
    //Note: IC cars may continue to drive for up to 100ms until valid readings are restored, but EVs must immediately cut power
    //Note: We need to decide how to report errors and how to perform actions when those errors occur.  For now, I'm calling an imaginary Err.Report function
    //-------------------------------------------------------------------
    if ((Sensor_TPS0.sensorValue < Sensor_TPS0.specMin) || (Sensor_TPS0.sensorValue > Sensor_TPS0.specMax))
    {
        //TODO: Err.Report(Err.Codes.TPS0Range, "TPS0 out of range:" & TPS0.sensorValue, Motor.Disable);
        //Note: We want to continue to run the rest of the code in this function to detect additional errors which we can report back to the dash.  To prevent the rest of the code from applying throttle, we take note that an error has occurred
        TPSErrorState++;
    }
  
    if ((Sensor_TPS1.sensorValue < Sensor_TPS1.specMin) || (Sensor_TPS1.sensorValue > Sensor_TPS1.specMax))
    {
        //TODO: Err.Report(Err.Codes.TPS1Range, "TPS1 out of range" & Pin140.sensorValue, Motor.Disable);
        TPSErrorState++;

    }
  
    //-------------------------------------------------------------------
    // If there are no issues at the pin level, then the next step
    // is to make sure the sensors actually agree with each other
    //-------------------------------------------------------------------
    //Calculate individual throttle percentages
    //Percent = (Voltage - CalibMin) / (CalibMax - CalibMin)
    float4 TPS0PedalPercent = getPercent(Sensor_TPS0.sensorValue, Sensor_TPS0.calibMin, Sensor_TPS0.calibMax, FALSE); //Analog Input 0
    float4 TPS1PedalPercent = getPercent(Sensor_TPS1.sensorValue, Sensor_TPS1.calibMin, Sensor_TPS1.calibMax, FALSE); //Analog input 1

    //Check for implausibility (discrepancy > 10%)
    //RULE: EV2.3.6 Implausibility is defined as a deviation of more than 10% pedal travel between the sensors.
    if (fabs(TPS1PedalPercent - TPS0PedalPercent) > .1)
    {
        //Err.Report(Err.Codes.TPSDiscrepancy, "TPS discrepancy of over 10%", Motor.Stop);
        TPSErrorState++;
    }

    //-------------------------------------------------------------------
    //Make sure the sensors have been calibrated
    //-------------------------------------------------------------------
    if ((Sensor_TPS0.isCalibrated == FALSE) || (Sensor_TPS1.isCalibrated == FALSE))
    {
        TPSErrorState++;
    }

    //What about other error states?
    //Voltage outside of calibration range
    //Voltages off center

    if (TPSErrorState > 0)
    {
        return 0;
    }
    else
    {
        return (TPS0PedalPercent + TPS1PedalPercent) / 2;
    }
}
 


/*****************************************************************************
    * BPS (Brake Pressure Sensor) functions
    * 
    ****************************************************************************/
//Input: Voltage
//Outputs: ???

/*-------------------------------------------------------------------
    * CalibrateBPS
    * Description: Records brake system pressure at point where wheels
    *              lock up [THIS IS A SUGGESTION - maybe we just use the
    *              DAQ to determine this and hardcode a value
    * Parameters:  
    * Inputs:      Pin151 = Analog Input 2
    * Returns:     
    * Notes:       
    * Throws:      
    -------------------------------------------------------------------*/
void CalibrateBPS(void){
    //TODO: Figure out how much braking effort (in hydraulic system PSI) = 100%
    //      braking effort (shooting for 100 lbs pedal effort?)
    //TODO: Use above value for 100% of "pedal travel"
    //      For now, assume 4.5V = 100 lbs 
    //TODO: Decide how often this must be done.  Could resting pressure vary
    //      with ambient temperature enough to mess up regen levels?
    //Note: It may not be feasible to measure max pressure that each driver will apply.  Instead, it may be best to go with absolute values (.5 to 4.5 V), or to at least record PSI or Voltage at zero pedal.  Not sure how this is affected by having a Sealed sensor (as opposed to Gage), but presumably brake force is SIGNIFICANTLY higher than ambient temperature anyway.
    //Sensor_BPS0.CalibMin = Sensor_BPS0.GetVoltage.Min;
    //Sensor_BPS0.CalibMax = Sensor_BPS0.GetVoltage.Max;
}

/*-------------------------------------------------------------------
    * GetBrakePosition
    * Description: Reads TPS Pin voltages and returns % of throttle pedal travel.
    * Parameters:  None
    * Inputs:      
    * Returns:     Brake pedal travel in percent (from 0 to 1), where
    *              0 = no pedal travel and 100 = //TODO: 4 locked wheels???
    *              NOTE: This means values above 100% are both allowed
    *                    and EXPECTED during e.g. brake tests & extreme lockups
    * Notes:       
    * Throws:      
    -------------------------------------------------------------------*/
void GetBrakePosition(void)
{
    //GetTPSValue = 0;  //This is the default value that this function will return
    //TPSErrorState = 0; //No errors have been detected so far.
    //-------------------------------------------------------------------
    //First check for implausibility at the pin level
    //USE SPEC SHEET VALUES, NOT CALIBRATION VALUES
    //RULE: EV2.4.4 - signal outside of operating range (EV2.4.5) requires
    //                immediate shutdown of power to motor
    //-------------------------------------------------------------------
    //if(Sensor_BPS.GetVoltage < Sensor_BPS.SpecMin Or Sensor_BPS.GetVoltage > Sensor_BPS.SpecMax
    {
    //Err.Report(Err.Codes.BPSRange, "BPS voltage out of range:" & Sensor_BPS.GetVoltage, Motor.Disable);
    //TPSErrorState++;
    }

    //-------------------------------------------------------------------
    // Calculate pedal travel
    //-------------------------------------------------------------------
    //Sensor_BPS.PedalPercent = Sensor_BPS.GetVoltage / Sensor_BPS.CalibMax;

    //If TPSErrorState > 0 Then
    //Reduce motor torque to zero
    //GetTPSValue already = 0;
    //Else
    //GetBrakePosition = Sensor_BPS.PedalPercent;
    //End If
}


/*****************************************************************************
* Wheel Speed Sensor functions
* FR = Pin275 = Digital Timer Input 0
* FL = Pin268 = Digital Timer Input 1
* RR = Pin274 = Digital Timer Input 2
* RL = Pin267 = Digital Timer Input 3
****************************************************************************/
//Inputs: Wheel speed sensor frequency
//Outputs: 4 speeds, RPM, ground speed
//4 funcs, one to get each wheel’s speed
//16 bumps per rotation, 16 hz = 1 rotation per second
#define NUM_BUMPS 16
#define WHEEL_DIAMETER 18 //Inches
#define NUM_BUMPS_D (double)NUM_BUMPS
#define WHEEL_DIAMETER_D (double)WHEEL_DIAMETER

double freq_to_rpm(double freq) {
    return (double)((freq/NUM_BUMPS_D)*60.0);
}

//Theoretical ground speed
//63360 inches per mile. Wish we could use metric.
double rpm_to_mph(double rpm) {
    return (double)((3.14159265*WHEEL_DIAMETER_D*rpm*60.0)/63360.0);
}


/*****************************************************************************
* Shock pot(iometer) functions - FOUR NEEDED
* FR = Pin150 = Analog Input 4
* FL = Pin138 = Analog Input 5
* RR = Pin149 = Analog Input 6
* RL = Pin137 = Analog Input 7
* 0 = ride height
****************************************************************************/
//Input: Ohms
//Outputs: ???
//See VCU Manual section 5.8.8 - there are different outputs depending on sensor resistance
//Example sensor: Active Sensors CLS0950
//Resistive range: 0.4 to 6.0 kohm
//ShockPot.


/*****************************************************************************
* Doppler speed sensor functions
* 
****************************************************************************/
//Input: Frequency
//Outputs: ???

/*****************************************************************************
* Water temp sensor functions
* 
****************************************************************************/
//Input: Ohms
//Outputs: ???

/*****************************************************************************
* Dash button functions
* 
****************************************************************************/
//TCS
//ECO
//RTD
//blablalba



/*****************************************************************************
* Error Reporting Stuff
*
* TODO: Write error reporting functions
****************************************************************************/
//Err.Report(ErrCode, ErrDesc, ErrAction) {
    //Write an error to the log
    //Add error code to dash
    //000 = TPS0 out of range
    //001 = TPS1 out of range
//}

