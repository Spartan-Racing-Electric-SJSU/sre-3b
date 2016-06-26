#include <stdlib.h>  //Needed for malloc
#include <math.h>
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
    //me->tps0 = (benchMode == TRUE) ? &Sensor_BenchTPS0 : &Sensor_TPS0;
    //me->tps1 = (benchMode == TRUE) ? &Sensor_BenchTPS1 : &Sensor_TPS1;
    me->tps0 = &Sensor_TPS0;
    me->tps1 = &Sensor_TPS1;

	//Where/should these be hardcoded?
	me->tps0_reverse = FALSE;
	me->tps1_reverse = TRUE;

    me->percent = 0;
    me->runCalibration = FALSE;  //Do not run the calibration at the next main loop cycle

    //me->calibrated = FALSE;
    //TorqueEncoder_resetCalibration(me);

    //Default calibration values
    me->tps0_calibMin = 1117;  //me->tps0->sensorValue;
    me->tps0_calibMax = 2304;  //me->tps0->sensorValue;
    me->tps1_calibMin = 2702;  //me->tps1->sensorValue;
    me->tps1_calibMax = 3890;  //me->tps1->sensorValue;
    //me->tps0_calibMin = 558;  //me->tps0->sensorValue;
    //me->tps0_calibMax = 2649;  //me->tps0->sensorValue;
    //me->tps1_calibMin = 2382;  //me->tps1->sensorValue;
    //me->tps1_calibMax = 4441;  //me->tps1->sensorValue;
    me->calibrated = TRUE;

    return me;
}

//Updates all values based on sensor readings, safety checks, etc
void TorqueEncoder_update(TorqueEncoder* me)
{
	me->tps0_value = me->tps0->sensorValue;
	me->tps1_value = me->tps1->sensorValue;

	me->percent = 0;
	ubyte2 errorCount = 0;
	
	//This function runs before the calibration cycle function.  If calibration is currently
	//running, then set the percentage to zero for safety purposes.
	if (me->runCalibration == TRUE)
	{
		errorCount++;  //DO SOMETHING WITH THIS
	}
	else
	{
		//getPedalTravel = 0;

		//-------------------------------------------------------------------
		// Make sure the sensors have been calibrated
		//-------------------------------------------------------------------
		//if ((Sensor_TPS0.isCalibrated == FALSE) || (Sensor_TPS1.isCalibrated == FALSE))
		if (me->calibrated == FALSE)
		{
			me->tps0_percent = 0;
			me->tps1_percent = 0;
			(errorCount)++;  //DO SOMETHING WITH THIS
		}
		else
		{
			//Calculate individual throttle percentages
			//Percent = (Voltage - CalibMin) / (CalibMax - CalibMin)
			//float4 TPS0PedalPercent = getPercent(TPS0_Val, TPS0_CalMin, TPS0_CalMax, TRUE); //Analog Input 0
			//float4 TPS1PedalPercent = getPercent(TPS1_Val, TPS1_CalMin, TPS1_CalMax, TRUE); //Analog input 1

			me->tps0_percent = getPercent(me->tps0_value, me->tps0_calibMin, me->tps0_calibMax, TRUE);

			//Todo: Fix percent calculator to make this work
			//me->tps1_percent = getPercent(me->tps1_value, me->tps1_calibMin, me->tps1_calibMax, TRUE);
			float4 range = me->tps1_calibMax - me->tps1_calibMin;
			float4 travel = me->tps1_calibMax - me->tps1_value;
			
			me->tps1_percent = travel / range;
			if (travel > range) { me->tps1_percent = 1; }
			if (travel > me->tps1_calibMax) { me->tps1_percent = 0; }

			//TorqueEncoder_plausibilityCheck(me, 0, &me->implausibility);
			/*if (me->implausibility == TRUE)
			{
				me->percent = 0;
			}
			else
			{*/
				me->percent = (me->tps0_percent + me->tps1_percent) / 2;
			//}
		}
	}
}

void TorqueEncoder_resetCalibration(TorqueEncoder* me)
{
    me->calibrated = FALSE;
    //me->tps0_rawCalibMin = me->tps0->specMax;
    //me->tps0_rawCalibMax = me->tps0->specMin;
    //me->tps0_calibMin = me->tps0->specMax;
	//me->tps0_calibMax = me->tps0->specMin;
    
    //me->tps1_rawCalibMin = me->tps1->specMax;
    //me->tps1_rawCalibMax = me->tps1->specMin;
	//me->tps1_calibMin = me->tps1->specMax;
	//me->tps1_calibMax = me->tps1->specMin;
    
    //Normal
    me->tps0_calibMin = me->tps0->sensorValue;
    me->tps0_calibMax = me->tps0->sensorValue;
    me->tps1_calibMin = me->tps1->sensorValue;
    me->tps1_calibMax = me->tps1->sensorValue;
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
    }
    else
    {
        IO_RTC_StartTime(&(me->timestamp_calibrationStart));  //extend the calibration time
    }
}

/*-------------------------------------------------------------------
* CalibrateTPS
* Description: Records TPS minimum/maximum voltages (when?) and stores them (where?), or flags that calibration is complete
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
        if (IO_RTC_GetTimeUS(me->timestamp_calibrationStart) < (ubyte4)(me->calibrationRunTime) * 1000 * 1000)
        {
			//The calibration itself
			if (me->tps0->sensorValue < me->tps0_calibMin) { me->tps0_calibMin = me->tps0->sensorValue; }
			if (me->tps0->sensorValue > me->tps0_calibMax) { me->tps0_calibMax = me->tps0->sensorValue; }

			if (me->tps1->sensorValue < me->tps1_calibMin) { me->tps1_calibMin = me->tps1->sensorValue; }
			if (me->tps1->sensorValue > me->tps1_calibMax) { me->tps1_calibMax = me->tps1->sensorValue; }

        }
        else  //Calibration shutdown
        {
            //90 degree sensor active range.. so just say % = degrees
            //float4 pedalTopPlay = 1.05;
            //float4 pedalBottomPlay = .95;

            //me->tps0_calibMin *= me->tps0_reverse ? pedalBottomPlay : pedalTopPlay;
            //me->tps0_calibMax *= me->tps0_reverse ? pedalTopPlay : pedalBottomPlay;
            //me->tps1_calibMin *= me->tps1_reverse ? pedalBottomPlay : pedalTopPlay;
            //me->tps1_calibMax *= me->tps1_reverse ? pedalTopPlay : pedalBottomPlay;

            //Shrink the calibrated range slightly
            float4 shrink0 = (me->tps0_calibMax - me->tps0_calibMin) * .05;
            float4 shrink1 = (me->tps1_calibMax - me->tps1_calibMin) * .05;
            me->tps0_calibMin += shrink0;
            me->tps0_calibMax -= shrink0;
            me->tps1_calibMin += shrink1;
            me->tps1_calibMax -= shrink1;


			me->runCalibration = FALSE;
			me->calibrated = TRUE;
			Light_set(Light_dashTCS, 0);

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


void TorqueEncoder_getIndividualSensorPercent(TorqueEncoder* me, ubyte1 sensorNumber, float4* percent)
{
	//Sensor* tps;
	//ubyte2 calMin;
	//ubyte2 calMax;

	switch (sensorNumber)
	{
	case 0:
		*percent = me->tps0_percent;
		//tps = me->tps0;
		//calMin = me->tps0_calibMin;
		//calMax = me->tps0_calibMax;
		break;
	case 1:
		*percent = me->tps1_percent;
		//tps = me->tps1;
		//calMin = me->tps1_calibMin;
		//calMax = me->tps1_calibMax;
		break;
	}
	//float4 TPS0PedalPercent = getPercent(me->tps0->sensorValue, calMin, calMax, TRUE); //Analog Input 0
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
	*pedalPercent = me->percent;

    //What about other error states?
    //Voltage outside of calibration range
    //Voltages off center

    //    if (errorCount > 0)
    //    {
    //        return 0;
    //    }
    //    else
    //    {
    //return (TPS0PedalPercent + TPS1PedalPercent) / 2;
    //    }
}
