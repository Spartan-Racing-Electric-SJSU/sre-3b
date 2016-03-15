#include <stdlib.h>  //Needed for malloc

#include "IO_RTC.h"

#include "torqueEncoder.h"
#include "mathFunctions.h"

#include "sensors.h"
extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
TorqueEncoder* TorqueEncoder_new(bool benchMode)
{
    TorqueEncoder* me = (TorqueEncoder*)malloc(sizeof(struct _TorqueEncoder));
    //me->bench = benchMode;

    //TODO: Make sure the main loop is running before doing this
    me->tps0 = (benchMode == TRUE) ? &Sensor_BenchTPS0 : &Sensor_TPS0;
    me->tps1 = (benchMode == TRUE) ? &Sensor_BenchTPS1 : &Sensor_TPS1;

    me->percent = 0;
    me->runCalibration = FALSE;  //Do not run the calibration at the next main loop cycle

    //me->calibrated = FALSE;
    TorqueEncoder_resetCalibration(me);

    return me;
}

void TorqueEncoder_getIndividualSensorPercent(TorqueEncoder* me, ubyte1 sensorNumber, float4* percent)
{
    Sensor* tps;
    ubyte2 calMin;
    ubyte2 calMax;

    switch (sensorNumber)
    {
    case 0:
        tps = me->tps0;
        calMin = me->tps0_calibMin;
        calMax = me->tps0_calibMax;
        break;
    case 1:
        tps = me->tps1;
        calMin = me->tps1_calibMin;
        calMax = me->tps1_calibMax;
        break;
    }
    float4 TPS0PedalPercent = getPercent(me->tps0->sensorValue, calMin, calMax, TRUE); //Analog Input 0
}

void TorqueEncoder_resetCalibration(TorqueEncoder* me)
{
    me->calibrated = FALSE;
    me->tps0_rawCalibMin = me->tps0->specMax;
    me->tps0_rawCalibMax = me->tps0->specMin;
    me->tps0_calibMin = me->tps0->specMax;
    me->tps0_calibMax = me->tps0->specMin;
    
    me->tps1_rawCalibMin = me->tps1->specMax;
    me->tps1_rawCalibMax = me->tps1->specMin;
    me->tps1_calibMin = me->tps1->specMax;
    me->tps1_calibMax = me->tps1->specMin;
}

void TorqueEncoder_saveCalibrationToEEPROM(TorqueEncoder* me)
{

}

void TorqueEncoder_loadCalibrationFromEEPROM(TorqueEncoder* me)
{

}

void TorqueEncoder_startCalibration(TorqueEncoder* me, ubyte1 secondsToRun)
{
    if (me->runCalibration == FALSE) //Ignore the button if calibration is already running
    {
        me->runCalibration = TRUE;
        TorqueEncoder_resetCalibration(me);
        me->calibrated = FALSE;
        IO_RTC_StartTime(&(me->timestamp_calibrationStart));
        me->calibrationRunTime = secondsToRun;

        dashLight_set(dash_EcoLight, TRUE);
    }
}

void TorqueEncoder_getPercent(TorqueEncoder* me)
{
    //Return the average of the two sensors 
}

/*-------------------------------------------------------------------
* CalibrateTPS
* Description: Records TPS minimum/maximum voltages (when?) and stores them (where?)
* Parameters:
* Inputs:
* Returns:
* Notes:
* Throws:
-------------------------------------------------------------------*/
// Physical pedal travel will only occur across the center (about 1/2) of the actual sensor's range of travel
// The rules (especially EV2.3.6) are written about % of PEDAL travel, not percent of sensor range, so we must calculate pedal travel by recording the min/max voltages at min/max throttle positions
void TorqueEncoder_calibrationCycle(TorqueEncoder* me, ubyte1* errorCount)
//THIS FUNCTION SHOULD NOT BE CALLED FROM MAIN
{
    if (me->runCalibration == TRUE)
    {
        if (IO_RTC_GetTimeUS(me->timestamp_calibrationStart) > me->calibrationRunTime * 1000 * 1000)
        {
            //Calibration shutdown:
            me->runCalibration = FALSE;
            me->calibrated = TRUE;
            dashLight_set(dash_EcoLight, FALSE);
        }
        else
        {
            //The calibration itself
            if (me->tps0->sensorValue < me->tps0_calibMin) { me->tps0_calibMin = me->tps0->sensorValue; }
            if (me->tps0->sensorValue > me->tps0_calibMax) { me->tps0_calibMax = me->tps0->sensorValue; }
            if (me->tps1->sensorValue < me->tps1_calibMin) { me->tps1_calibMin = me->tps1->sensorValue; }
            if (me->tps1->sensorValue > me->tps1_calibMax) { me->tps1_calibMax = me->tps1->sensorValue; }
        }

    }
    else
    {
        //TODO: Throw warning: calibrationCycle helper function was called but calibration should not be running
    }

    //TODO: Write calibration data to EEPROM

    //TODO: Check for valid/reasonable calibration data

    //TODO: Do something on the display to show that voltages are being recorded

    //Idea: Display "bars" filling up on right segment (for gas pedal) _=E=_=E...
    //      Once calibration data makes sense, show pedal location (0-10%, 10-90%, 90-100%) with bars

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
void TorqueEncoder_getPedalTravel(TorqueEncoder* me, ubyte1* errorCount, float4* pedalPercent)
{
    *errorCount = 0; //No errors have been detected so far.
    //getPedalTravel = 0;
    bool implausibility;

    //-------------------------------------------------------------------
    // Make sure the sensors have been calibrated
    //-------------------------------------------------------------------
    if ((Sensor_TPS0.isCalibrated == FALSE) || (Sensor_TPS1.isCalibrated == FALSE))
    {
        (*errorCount)++;
    }
    else
    {
        me->tps0_percent = getPercent(me->tps0->sensorValue, me->tps0_calibMin, me->tps0_calibMax, TRUE);
        me->tps1_percent = getPercent(me->tps0->sensorValue, me->tps1_calibMin, me->tps1_calibMax, TRUE);

        TorqueEncoder_plausibilityCheck(me, &implausibility);
        if (implausibility == TRUE)
        {
            getPedalTravel = 0;
        }
        else
        {

        }
    }


    //What about other error states?
    //Voltage outside of calibration range
    //Voltages off center

    //    if (errorCount > 0)
    //    {
    //        return 0;
    //    }
    //    else
    //    {
    return (TPS0PedalPercent + TPS1PedalPercent) / 2;
    //    }
}

void TorqueEncoder_plausibilityCheck(TorqueEncoder* me, ubyte1* errorCount, bool* fail)
{
    //-------------------------------------------------------------------
    //First check for implausibility at the pin level
    //USE SPEC SHEET VALUES, NOT CALIBRATION VALUES
    //RULE: EV2.3.10 - signal outside of operating range is considered a failure
    //Note: IC cars may continue to drive for up to 100ms until valid readings are restored, but EVs must immediately cut power
    //Note: We need to decide how to report errors and how to perform actions when those errors occur.  For now, I'm calling an imaginary Err.Report function
    //-------------------------------------------------------------------
    if ((TPS0_Val < TPS0_Min) || (TPS0_Val > TPS0_Max))
    {
        //TODO: Err.Report(Err.Codes.TPS0Range, "TPS0 out of range:" & TPS0.sensorValue, Motor.Disable);
        //Note: We want to continue to run the rest of the code in this function to detect additional errors which we can report back to the dash.  To prevent the rest of the code from applying throttle, we take note that an error has occurred
        (*errorCount)++;
    }

    if ((TPS1_Val < TPS1_Min) || (TPS1_Val > TPS1_Max))
    {
        //TODO: Err.Report(Err.Codes.TPS1Range, "TPS1 out of range" & Pin140.sensorValue, Motor.Disable);
        (*errorCount)++;

    }

    //-------------------------------------------------------------------
    // If there are no issues at the pin level, then the next step
    // is to make sure the sensors actually agree with each other
    //-------------------------------------------------------------------
    //Calculate individual throttle percentages
    //Percent = (Voltage - CalibMin) / (CalibMax - CalibMin)
    float4 TPS0PedalPercent = getPercent(TPS0_Val, TPS0_CalMin, TPS0_CalMax, TRUE); //Analog Input 0
    float4 TPS1PedalPercent = getPercent(TPS1_Val, TPS1_CalMin, TPS1_CalMax, TRUE); //Analog input 1

                                                                                    //Check for implausibility (discrepancy > 10%)
                                                                                    //RULE: EV2.3.6 Implausibility is defined as a deviation of more than 10% pedal travel between the sensors.
    if (fabs(TPS1PedalPercent - TPS0PedalPercent) > .1)
    {
        //Err.Report(Err.Codes.TPSDiscrepancy, "TPS discrepancy of over 10%", Motor.Stop);
        (*errorCount)++;
    }

    return FALSE;
}