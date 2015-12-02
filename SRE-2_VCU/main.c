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
#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_RTC.h"
#include "IO_CAN.h"
#include "IO_DIO.h"
#include "IO_PWM.h"

//Our code
#include "vcuInitializations.c"
#include "sensors.c"
#include "outputCalculations.c"
#include "motorController.c"


/*****************************************************************************
* Main!
* Initializes I/O
* Contains sensor polling loop (always running)
****************************************************************************/
void main(void)
{
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


