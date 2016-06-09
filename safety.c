#include <stdlib.h>  //Needed for malloc
//#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"

#include "safety.h"
#include "mathFunctions.h"

#include "sensors.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"

#include "motorController.h"
#include "bms.h"
#include "serial.h"

//last flag is 0x 8000 0000 (32 flags)
//Faults
static const ubyte4 tpsOutOfRange = 1;
static const ubyte4 bpsOutOfRange = 2;
static const ubyte4 tpsPowerFailure = 4;
static const ubyte4 bpsPowerFailure = 8;

static const ubyte4 tpsSignalFailure = 0x10;
static const ubyte4 bpsSignalFailure = 0x20;
static const ubyte4 tpsNotCalibrated = 0x40;
static const ubyte4 bpsNotCalibrated = 0x80;

static const ubyte4 tpsOutOfSync = 0x100;
static const ubyte4 bpsOutOfSync = 0x200; //NOT USED
static const ubyte4 tpsbpsImplausible = 0x400;
//static const ubyte4 UNUSED = 0x800;

//static const ubyte4 tpsOutOfSync = 0x1000;
//static const ubyte4 bpsOutOfSync = 0x2000; //NOT USED
//static const ubyte4 tpsbpsImplausible = 0x4000;
//static const ubyte4 UNUSED = 0x8000;




//Warnings
static const ubyte4 LVSBatteryLow = 1;


static const ubyte4 HVILTermSenseLost = 0x10;


/*****************************************************************************
* SafetyChecker object
******************************************************************************
* ToDo: change to ubyte1[8] (64 flags possible)
* 1 = fault
* 0 = no fault
****************************************************************************/
struct _SafetyChecker {
	//Problems that require motor torque to be disabled
    SerialManager* serialMan;
    ubyte4 faults;
    ubyte4 warnings;
};

/*****************************************************************************
* Torque Encoder (TPS) functions
* RULE EV2.3.5:
* If an implausibility occurs between the values of these two sensors the power to the motor(s) must be immediately shut down completely.
* It is not necessary to completely deactivate the tractive system, the motor controller(s) shutting down the power to the motor(s) is sufficient.
****************************************************************************/
SafetyChecker* SafetyChecker_new(SerialManager* sm)
{
    SafetyChecker* me = (SafetyChecker*)malloc(sizeof(struct _SafetyChecker));

    me->serialMan = sm;
    SerialManager_send(me->serialMan, "SafetyChecker's reference to SerialManager was created.\n");
	//Initialize all safety checks to FAIL? Not anymore
    me->faults = 0;
    me->warnings = 0;
    return me;
}

//Updates all values based on sensor readings, safety checks, etc
void SafetyChecker_update(SafetyChecker* me, TorqueEncoder* tps, BrakePressureSensor* bps, Sensor* HVILTermSense, Sensor* LVBattery)
{
    SerialManager_send(me->serialMan, "Entered SafetyChecker_update().\n");
    /*****************************************************************************
    * Faults
    ****************************************************************************/
	//===================================================================
	// Get calibration status
	//===================================================================
    me->faults = 0;
    if (tps->calibrated == FALSE) { me->faults |= tpsNotCalibrated; }
    if (bps->calibrated == FALSE) { me->faults |= bpsNotCalibrated; }

	//===================================================================
	// Check if VCU was able to get a reading
	//===================================================================
	if (tps->tps0->ioErr_powerInit != IO_E_OK
		|| tps->tps1->ioErr_powerInit != IO_E_OK
		|| tps->tps0->ioErr_powerSet != IO_E_OK
		|| tps->tps1->ioErr_powerSet != IO_E_OK)
    {
        me->faults |= tpsPowerFailure;
    }
    else
    {
        me->faults &= ~tpsPowerFailure;
    }

    
	if (   tps->tps0->ioErr_signalInit != IO_E_OK
		|| tps->tps1->ioErr_signalInit != IO_E_OK
		|| tps->tps0->ioErr_signalGet != IO_E_OK
		|| tps->tps1->ioErr_signalGet != IO_E_OK)
	{
		//me->faults |= tpsSignalFailure;
        SerialManager_send(me->serialMan, "TPS signal error\n");
	}
    else
    {
        me->faults &= ~tpsSignalFailure;
    }

	if (bps->bps0->ioErr_powerInit != IO_E_OK
		|| bps->bps0->ioErr_powerSet != IO_E_OK)
	{
		me->faults |= bpsPowerFailure;
	}
    else
    {
        me->faults &= ~bpsPowerFailure;
    }

	if (bps->bps0->ioErr_signalInit != IO_E_OK
		|| bps->bps0->ioErr_signalGet != IO_E_OK)
	{
		me->faults |= bpsSignalFailure;
	}
    else
    {
        me->faults &= ~bpsSignalFailure;
    }

	//===================================================================
	// Make sure raw sensor readings are within operating range
	//===================================================================
	//RULE: EV2.3.10 - signal outside of operating range is considered a failure
	//  This refers to SPEC SHEET values, not calibration values
	//Note: IC cars may continue to drive for up to 100ms until valid readings are restored, but EVs must immediately cut power
	//Note: We need to decide how to report errors and how to perform actions when those errors occur.  For now, I'm calling an imaginary Err.Report function
	
	//-------------------------------------------------------------------
	//Torque Encoder
	//-------------------------------------------------------------------
	if (tps->tps0->sensorValue < tps->tps0->specMin || tps->tps0->sensorValue > tps->tps0->specMax
	||  tps->tps1->sensorValue < tps->tps1->specMin || tps->tps1->sensorValue > tps->tps1->specMax)
	{
		me->faults |= tpsOutOfRange;
	}
    else
    {
        me->faults &= ~tpsOutOfRange;
    }

	//-------------------------------------------------------------------
	//Brake Pressure Sensor
	//-------------------------------------------------------------------
	if (bps->bps0->sensorValue < bps->bps0->specMin || bps->bps0->sensorValue > bps->bps0->specMax)
	{
		me->faults |= bpsOutOfRange;
	}
    else
    {
        me->faults &= ~bpsOutOfRange;
    }

	//===================================================================
	// Make sure calibrated TPS readings are in sync with each other
	//===================================================================
	// EV2.3.5 If an implausibility occurs between the values of these two sensors
	//  the power to the motor(s) must be immediately shut down completely. It is not necessary 
	//  to completely deactivate the tractive system, the motor controller(s) shutting down the 
	//  power to the motor(s) is sufficient.
	// EV2.3.6 Implausibility is defined as a deviation of more than 10 % pedal travel between the sensors.
	//-------------------------------------------------------------------
	
	//Check for implausibility (discrepancy > 10%)
	//RULE: EV2.3.6 Implausibility is defined as a deviation of more than 10% pedal travel between the sensors.
	float4 tps0Percent;   //Pedal percent float (a decimal between 0 and 1
	float4 tps1Percent;

	TorqueEncoder_getIndividualSensorPercent(tps, 0, &tps0Percent); //borrow the pedal percent variable
	TorqueEncoder_getIndividualSensorPercent(tps, 1, &tps1Percent);

    ubyte1* t0p[20];
    ubyte1* t1p[20];
    sprintf(t0p, "TPS0: %f\n", tps0Percent);
    sprintf(t1p, "TPS1: %f\n", tps1Percent);
    SerialManager_send(me->serialMan, t0p);
    SerialManager_send(me->serialMan, t1p);

	if ((tps1Percent - tps0Percent) > .1 || (tps1Percent - tps0Percent) < -.1)  //Note: Individual TPS readings don't go negative, otherwise this wouldn't work
	{

		//Err.Report(Err.Codes.TPSDiscrepancy, "TPS discrepancy of over 10%", Motor.Stop);
        SerialManager_send(me->serialMan, "TPS discrepancy of over 10%\n");

        me->faults |= tpsOutOfSync;
	}
    else
    {
        me->faults &= ~tpsOutOfSync;
    }



	//===================================================================
	//Torque Encoder <-> Brake Pedal Plausibility Check
	//===================================================================
	// EV2.5 Torque Encoder / Brake Pedal Plausibility Check
	//  The power to the motors must be immediately shut down completely, if the mechanical brakes 
	//  are actuated and the torque encoder signals more than 25 % pedal travel at the same time.
	//  This must be demonstrated when the motor controllers are under load.
	// EV2.5.1 The motor power shut down must remain active until the torque encoder signals less than 5 % pedal travel,
	//  no matter whether the brakes are still actuated or not.
	//-------------------------------------------------------------------
	//Implausibility if..
	if (bps->percent > .02 && tps->percent > .25) //If mechanical brakes actuated && tps > 25%
	{
		me->faults |= tpsbpsImplausible;
		//From here, assume that motor controller will check for implausibility before accepting commands
	}

	//Clear implausibility if...
	if ((me->faults & tpsbpsImplausible) > 0)
	{
		if (tps->percent < .05) //TPS is reduced to < 5%
		{
			me->faults &= ~tpsbpsImplausible;  //turn off the implausibility flag
		}
	}




    /*****************************************************************************
    * Warnings
    ****************************************************************************/
    //===================================================================
    // LVS Battery Check
    //===================================================================
    //  IO_ADC_UBAT: 0..40106  (0V..40.106V)
    //-------------------------------------------------------------------
    if (LVBattery->sensorValue <= 13100)
    {
        me->warnings |= LVSBatteryLow;
    }
    else
    {
        me->warnings &= ~LVSBatteryLow;
    }




    //===================================================================
    // HVIL Term Sense Check
    //===================================================================
    // If HVIL term sense goes low (because HV went down), motor torque
    // command should be set to zero before turning off the controller
    //-------------------------------------------------------------------
    if (HVILTermSense->sensorValue == FALSE)
    {
        me->warnings |= HVILTermSenseLost;
    }
    else
    {
        me->warnings &= ~HVILTermSenseLost;
    }
}


//Updates all values based on sensor readings, safety checks, etc
bool SafetyChecker_allSafe(SafetyChecker* me)
{
    return (me->faults == 0);
}

//Updates all values based on sensor readings, safety checks, etc
ubyte4 SafetyChecker_getFaults(SafetyChecker* me)
{
    return (me->faults);
}


//Updates all values based on sensor readings, safety checks, etc
ubyte4 SafetyChecker_getWarnings(SafetyChecker* me)
{
    return (me->warnings);
}

void SafetyChecker_ReduceTorque(SafetyChecker* me, MotorController* mcm0)
{
    if (me->faults > 0)
    {
        MCM_commands_setTorque(mcm0, 0);
        //MCM_commands_setTorqueLimit(1)
    }
}

////Updates all values based on sensor readings, safety checks, etc
//bool SafetyChecker_getError(SafetyChecker* me, SafetyCheck check)
//{
//	bool status;
//	switch (check)
//	{
//	case CHECK_tpsOutOfRange:
//		status = me->faults && tpsOutOfRange == 0;
//		break;
//
//	case CHECK_bpsOutOfRange:
//		status = me->faults && bpsOutOfRange;
//		break;
//
//	case CHECK_tpsNotCalibrated:
//		status = me->tpsNotCalibrated;
//		break;
//
//	case CHECK_bpsNotCalibrated:
//		status = me->bpsNotCalibrated;
//		break;
//
//	case CHECK_tpsOutOfSync:
//		status = me->tpsOutOfSync;
//		break;
//
//	case CHECK_tpsbpsImplausible:
//		status = me->tpsbpsImplausible;
//		break;
//
//	default:
//		status = TRUE; //error
//		break;
//	}
//
//	return status;
//}
//
////Updates all values based on sensor readings, safety checks, etc
//ubyte1 SafetyChecker_getErrorByte(SafetyChecker* me, ubyte1* errorSet)
//{
//	ubyte1 errorByte = 0;
//	switch ((ubyte2)errorSet)
//	{
//	case 0:
//		for (int bit = 0; bit <= 7; bit++)
//		{
//			errorByte <<= 1;  //Always leftshift first
//			switch (bit)
//			{
//			case 0: errorByte |= me->tpsOutOfRange ? 1 : 0; break;
//			case 1: errorByte |= me->tpsNotCalibrated ? 1 : 0; break;
//			case 2: errorByte |= me->tpsOutOfSync ? 1 : 0; break;
//			case 3: errorByte |= 0; break;
//			case 4: errorByte |= me->bpsOutOfRange ? 1 : 0; break;
//			case 5: errorByte |= me->bpsNotCalibrated ? 1 : 0; break;
//			case 6: errorByte |= 0; break;
//			case 7: errorByte |= me->tpsbpsImplausible ? 1 : 0; break;
//			default: break;
//			}
//		}
//		break;
//
//	default:
//		errorByte = 0xFF; //error
//		break;
//	}
//
//	return errorByte;
//}

//-------------------------------------------------------------------
// 80kW Limit Check
//-------------------------------------------------------------------
//Change this to return a multiplier instead of torque value
ubyte2 checkPowerDraw(BatteryManagementSystem* bms, MotorController* mcm )
{
	ubyte2 torqueThrottle = 0;
	
	// if either the bms or mcm goes over 75kw, limit torque 
	if((BMS_getPower(bms) > 75000) || (MCM_getPower(mcm) > 75000))
	{
		// using bmsPower since closer to e-meter
	    torqueThrottle = MCM_getCommandedTorque(mcm) - (((BMS_getPower(bms) - 80000)/80000) * MCM_getCommandedTorque(mcm));
	}
	
	return torqueThrottle;
}

//-------------------------------------------------------------------
// Pack Temp Check
//-------------------------------------------------------------------
void checkBatteryPackTemp(BatteryManagementSystem* bms)
{
	
	if((BMS_getPackTemp(bms) > 35))
	{
		// Turn on FANS
		//IO_DO_Init(IO_DO_06); 
		IO_DO_Set(IO_DO_06, TRUE); //pin 142 - sending 12V 
		
	}
	else
	{
		IO_DO_Set(IO_DO_06, FALSE);
	}
		
		
}