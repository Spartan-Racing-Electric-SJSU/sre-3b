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
#include "IO_ADC.h"
#include "IO_PWD.h"
#include "IO_PWM.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "mathFunctions.h"

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
extern Sensor Sensor_LVBattery;

extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSSwitchA;
extern Sensor Sensor_TCSSwitchB;
extern Sensor Sensor_HVILTerminationSense;

/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
//----------------------------------------------------------------------------
// Read sensors values from ADC channels
// The sensor values should be stored in sensor objects.
//----------------------------------------------------------------------------
void sensors_updateSensors(void)
{
    //TODO: Handle errors (using the return values for these Get functions)

    //TODO: RTDS

    //Torque Encoders ---------------------------------------------------
    IO_PWD_FreqGet(IO_PWM_00, &Sensor_TPS0.sensorValue);
    IO_PWD_FreqGet(IO_PWM_01, &Sensor_TPS1.sensorValue);

    //Brake Position Sensor ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_02, &Sensor_BPS0.sensorValue, &Sensor_BPS0.fresh);

    //?? - For future use ---------------------------------------------------
    //IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

    //Bench TPS ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_00, &Sensor_BenchTPS0.sensorValue, &Sensor_BenchTPS0.fresh);
    IO_ADC_Get(IO_ADC_5V_01, &Sensor_BenchTPS1.sensorValue, &Sensor_BenchTPS1.fresh);

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

    //Switches / Digital ---------------------------------------------------
    IO_DI_Get(IO_DI_00, &Sensor_RTDButton.sensorValue);
    IO_DI_Get(IO_DI_01, &Sensor_EcoButton.sensorValue);
    IO_DI_Get(IO_DI_02, &Sensor_TCSSwitchA.sensorValue);
    IO_DI_Get(IO_DI_03, &Sensor_TCSSwitchB.sensorValue);
    IO_DI_Get(IO_DI_07, &Sensor_HVILTerminationSense.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);


}

//----------------------------------------------------------------------------
//Testing MCM relay control
//----------------------------------------------------------------------------
void setMCMRelay(bool turnOn)
{
    IO_DO_Set(IO_DO_00, turnOn);
}

/*****************************************************************************
* Output Calculations
******************************************************************************
* Takes properties from devices (such as raw sensor values [ohms, voltage],
* MCU/BMS CAN messages, etc), performs calculations with that data, and updates
* the relevant objects' properties.
*
* This includes sensor calculations, motor controller control calculations,
* traction control, BMS/safety calculations, etc.
* (May need to split this up later)
*
* For example: GetThrottlePosition() takes the raw TPS voltages from the TPS
* sensor objects and returns the throttle pedal percent.  This function does
* NOT update the sensor objects, but it would be acceptable for another
* function in this file to do so.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-11-16 - Rusty Pedrosa -
*****************************************************************************/
/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
TorqueEncoder* TorqueEncoder_new(bool benchMode)
{
    TorqueEncoder* me = (TorqueEncoder*)malloc(sizeof(struct _TorqueEncoder));
    me->bench = benchMode;

    //TODO: Make sure the main loop is running before doing this
    me->tps0 = (bench == TRUE) ? &Sensor_BenchTPS0 : &Sensor_TPS0;
    me->tps1 = (bench == TRUE) ? &Sensor_BenchTPS1 : &Sensor_TPS1;

    me->percent = 0;
    me->runCalibration = FALSE;
    me->calibrated = FALSE;
}

void TorqueEncoder_resetCalibration(TorqueEncoder* me)
{
    me->calibrated = FALSE;
    tps0_rawCalibMin = tps0->specMax;
    tps0_rawCalibMax = tps0->specMin;
    tps0_calibMin = tps0->specMax;
    tps0_calibMax = tps0->specMin;

    tps1_rawCalibMin = tps1->specMax;
    tps1_rawCalibMax = tps1->specMin;
    tps1_calibMin = tps1->specMax;
    tps1_calibMax = tps1->specMin;
}

void TorqueEncoder_saveCalibrationToEEPROM(TorqueEncoder* me)
{

}

void TorqueEncoder_loadCalibrationFromEEPROM(TorqueEncoder* me)
{

}

void TorqueEncoder_startCalibration(TorqueEncoder* me, ubyte1 secondsToRun)
{
    TorqueEncoder_resetCalibration(me);
    me->calibrated = FALSE;
    me->runCalibration = TRUE;
    IO_RTC_StartTime(&(tps->timestamp_calibrationStart));
    tps->calibrationRunTime = secondsToRun;
}

void TorqueEncoder_getPercent(TorqueEncoder* me)
{
    TorqueEncoder_resetCalibration(me);
    me->calibrated = FALSE;
    me->runCalibration = TRUE;
    IO_RTC_StartTime(&(tps->timestamp_calibrationStart));
    tps->calibrationRunTime = secondsToRun;
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
void TorqueEncoder_calibrationCycle(TorqueEncoder* me)
//THIS FUNCTION SHOULD NOT BE CALLED FROM MAIN
{
    if (me->runCalibration == TRUE)
    {
        if (IO_RTC_GetTimeUS(me->timestamp_calibrationStart) > me->calibrationRunTime * 1000 * 1000)
        {
            me->runCalibration = FALSE;
            me->calibrated = TRUE;
        }
        else
        {
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
    getPedalTravel = 0;
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