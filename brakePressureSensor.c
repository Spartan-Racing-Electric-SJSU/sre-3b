#include <stdlib.h>  //Needed for malloc
#include <math.h>
#include "IO_RTC.h"

#include "brakePressureSensor.h"
#include "mathFunctions.h"

#include "sensors.h"
//extern Sensor Sensor_BPS0;
//extern Sensor Sensor_BenchTPS1;

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
BrakePressureSensor* BrakePressureSensor_new(void)
{
    BrakePressureSensor* me = (BrakePressureSensor*)malloc(sizeof(struct _BrakePressureSensor));
    //me->bench = benchMode;

    //TODO: Make sure the main loop is running before doing this
    me->bps0 = &Sensor_BPS0;
    //me->tps1 = (benchMode == TRUE) ? &Sensor_BenchTPS1 : &Sensor_TPS1;
	
	//Where/should these be hardcoded?
	me->bps0_reverse = FALSE;
	//me->bps1_reverse = TRUE;

    me->percent = 0;
    me->runCalibration = FALSE;  //Do not run the calibration at the next main loop cycle

    //me->calibrated = FALSE;
    BrakePressureSensor_resetCalibration(me);

    return me;
}

//Updates all values based on sensor readings, safety checks, etc
void BrakePressureSensor_update(BrakePressureSensor* me)
{
	me->bps0_value = me->bps0->sensorValue;
	//me->bps1_value = me->bps1->sensorValue;

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
			(errorCount)++;  //DO SOMETHING WITH THIS
		}
		else
		{
			me->bps0_percent = getPercent(me->bps0_value, me->bps0_calibMin, me->bps0_calibMax, TRUE);

			//me->bps1_percent = getPercent(me->bps1_value, me->bps1_calibMin, me->bps1_calibMax, TRUE);
			/*float4 range = me->bps1_calibMax - me->bps1_calibMin;
			float4 travel = me->bps1_calibMax - me->bps1_value;
			
			me->bps1_percent = travel / range;
			if (travel > range) { me->bps1_percent = 1; }
			if (travel > me->bps1_calibMax) { me->bps1_percent = 0; }
			*/


			//BrakePressureSensor_plausibilityCheck(me, 0, &me->implausibility);
			/*if (me->implausibility == TRUE)
			{
				me->percent = 0;
			}
			else
			{*/
			me->percent = me->bps0_percent; //(me->bps0_percent + me->bps1_percent) / 2;
			//}
		}
	}
}

void BrakePressureSensor_resetCalibration(BrakePressureSensor* me)
{
    me->calibrated = FALSE;
    //me->bps0_rawCalibMin = me->bps0->specMax;
    //me->bps0_rawCalibMax = me->bps0->specMin;
    //me->bps0_calibMin = me->bps0->specMax;
	//me->bps0_calibMax = me->bps0->specMin;
	me->bps0_calibMin = me->bps0->sensorValue;
	me->bps0_calibMax = me->bps0->sensorValue;

    //me->bps1_rawCalibMin = me->bps1->specMax;
    //me->bps1_rawCalibMax = me->bps1->specMin;
	//me->bps1_calibMin = me->bps1->specMax;
	//me->bps1_calibMax = me->bps1->specMin;
//	me->bps1_calibMin = me->bps1->sensorValue;
//	me->bps1_calibMax = me->bps1->sensorValue;
}

void BrakePressureSensor_saveCalibrationToEEPROM(BrakePressureSensor* me)
{

}

void BrakePressureSensor_loadCalibrationFromEEPROM(BrakePressureSensor* me)
{

}

void BrakePressureSensor_startCalibration(BrakePressureSensor* me, ubyte1 secondsToRun)
{
    if (me->runCalibration == FALSE) //Ignore the button if calibration is already running
    {
        me->runCalibration = TRUE;
        BrakePressureSensor_resetCalibration(me);
        me->calibrated = FALSE;
        IO_RTC_StartTime(&(me->timestamp_calibrationStart));
        me->calibrationRunTime = secondsToRun;

		dashLight_set(dash_EcoLight, TRUE);
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
void BrakePressureSensor_calibrationCycle(BrakePressureSensor* me, ubyte1* errorCount)
{
    if (me->runCalibration == TRUE)
    {
        if (IO_RTC_GetTimeUS(me->timestamp_calibrationStart) < (ubyte4)(me->calibrationRunTime) * 1000 * 1000)
        {
			//The calibration itself
			if (me->bps0->sensorValue < me->bps0_calibMin) { me->bps0_calibMin = me->bps0->sensorValue; }
			if (me->bps0->sensorValue > me->bps0_calibMax) { me->bps0_calibMax = me->bps0->sensorValue; }

			//if (me->bps1->sensorValue < me->bps1_calibMin) { me->bps1_calibMin = me->bps1->sensorValue; }
			//if (me->bps1->sensorValue > me->bps1_calibMax) { me->bps1_calibMax = me->bps1->sensorValue; }

        }
        else  //Calibration shutdown
        {
			////If the sensor goes in reverse direction then flip the min/max values
			//if (me->bps0_reverse == TRUE)
			//{
			//	float4 temp = me->bps0_calibMin;
			//	me->bps0_calibMin = me->bps0_calibMax;
			//	me->bps0_calibMax = temp;
			//}

			//if (me->bps1_reverse == TRUE)
			//{
			//	float4 temp = me->bps1_calibMin;
			//	me->bps1_calibMin = me->bps1_calibMax;
			//	me->bps1_calibMax = temp;
			//}

			me->runCalibration = FALSE;
			me->calibrated = TRUE;
			dashLight_set(dash_TCSLight, FALSE);
			

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


void BrakePressureSensor_getIndividualSensorPercent(BrakePressureSensor* me, ubyte1 sensorNumber, float4* percent)
{
	//Sensor* bps;
	//ubyte2 calMin;
	//ubyte2 calMax;

	switch (sensorNumber)
	{
	case 0:
		*percent = me->bps0_percent;
		//bps = me->bps0;
		//calMin = me->bps0_calibMin;
		//calMax = me->bps0_calibMax;
		break;
	case 1:
		*percent = 0;//me->bps1_percent;
		//bps = me->bps1;
		//calMin = me->bps1_calibMin;
		//calMax = me->bps1_calibMax;
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
void BrakePressureSensor_getPedalTravel(BrakePressureSensor* me, ubyte1* errorCount, float4* pedalPercent)
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
