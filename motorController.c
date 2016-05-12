#include <stdlib.h>  //Needed for malloc
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"
#include "sensorCalculations.h"

#include "torqueEncoder.h"
#include "readyToDriveSound.h"


extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSSwitchA;
extern Sensor Sensor_TCSSwitchB;
extern Sensor Sensor_HVILTerminationSense;

/*****************************************************************************
 * Motor Controller (MCM)
 ******************************************************************************
 *
 ****************************************************************************/

struct _MotorController {
	//----------------------------------------------------------------------------
	// Controller statuses/properties
	//----------------------------------------------------------------------------
	// These represent the state of the controller (set at run time, not compile
	// time.)  These are updated by canInput.c
	//----------------------------------------------------------------------------
	ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller
	ubyte4 timeStamp_inverterEnabled;
	ubyte2 torqueMaximum;  //Max torque that can be commanded in deciNewton*meters ("100" = 10.0 Nm)

	ubyte1 startupStage;
	Status lockoutStatus;
	Status inverterStatus;
	//bool startRTDS;
	/*ubyte4 vsmStatus0;      //0xAA Byte 0,1
    ubyte4 vsmStatus1;      //0xAA Byte 0,1
    ubyte4 vsmStatus2;      //0xAA Byte 0,1
    ubyte4 vsmStatus3;      //0xAA Byte 0,1
    ubyte4 faultCodesPOST; //0xAB Byte 0-3
    ubyte4 faultCodesRUN;  //0xAB Byte 4-7*/

	//----------------------------------------------------------------------------
	// Control parameters
	//----------------------------------------------------------------------------
	// These are updated by ??? and will be sent to the VCU over CAN
	//----------------------------------------------------------------------------
	//struct _commands {
	ubyte4 timeStamp_lastCommandSent;  //from IO_RTC_StartTime(&)
	ubyte2 updateCount; //Number of updates since lastCommandSent

	ubyte2 commands_torque;
	ubyte2 commands_torqueLimit;
	ubyte1 commands_direction;

	sbyte2 motor_temp;
	sbyte2 DC_Voltage;
	sbyte2 DC_Current;

	sbyte2 commandedTorque;
	ubyte2 currentPower;

	sbyte2 motorRPM;
	//unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
	Status commands_discharge;
	Status commands_inverter;
	//ubyte1 controlSwitches; // example: 0b00000001 = inverter is enabled, discharge is disabled.

};

/*
    //----------------------------------------------------------------------------
    // Control functions
    //----------------------------------------------------------------------------
    void(*setTorque)(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
    void(*setDirection)(MotorController* me, Direction rotation);
    void(*setInverter)(MotorController* me, Status inverterState);
    void(*setDischarge)(MotorController* me, Status dischargeState);
    void(*setTorqueLimit)(MotorController* me, ubyte2 torqueLimit);
    //        void(*motorController_setTorque)(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
    //        void(*motorController_setDirection)(MotorController* me, Direction rotation);
    //        void(*motorController_setInverter)(MotorController* me, Status inverterState);
    //        void(*motorController_setDischarge)(MotorController* me, Status dischargeState);
    //        void(*motorController_setTorqueLimit)(MotorController* me, ubyte2 torqueLimit);
    void(*updateLockoutStatus)(MotorController* me, Status newState);
    void(*updateInverterStatus)(MotorController* me, Status newState);
    void(*getLockoutStatus)(MotorController* me);
    void(*getInverterStatus)(MotorController* me);
    }
 */
//_commands commands;
//};



void mcm_parseCanMessage(MotorController* mcm, IO_CAN_DATA_FRAME* mcmCanMessage)
{
    switch (mcmCanMessage->id)
    {
    case 0x0A0:
        //0,1 module A temperature
        //2,3 module B temperature
        //4,5 module C temperature
        //6,7 gate driver board temperature
        break;

    case 0x0A1:
        //0,1 control board temp
        //2,3 rtd 1 temp
        //4,5 rtd 2 temp
        //6,7 rtd 3 temp
        break;

    case 0x0A2:
        //0,1 rtd 4 temp
        //2,3 rtd 5 temp
        //4,5 motor temperature***
        mcm->motor_temp = ((mcmCanMessage->data[4] << 8) | (mcmCanMessage->data[5]));
        //6,7 torque shudder
        break;

    case 0x0A3:
        //0,1 voltage analog input #1
        //2,3 voltage analog input #2
        //4,5 voltage analog input #3
        //6,7 voltage analog input #4
        break;

    case 0x0A4:
        // booleans //
        // 0 digital input #1
        // 1 digital input #2
        // 2 digital input #3
        // 4 digital input #5
        // 5 digital input #6
        // 6 digital input #7
        // 7 digital input #8
        break;

    case 0x0A5:
        //0,1 motor angle (electrical)
        //2,3 motor speed*** // in rpms
        mcm->motorRPM = ((mcmCanMessage->data[2] << 8) | (mcmCanMessage->data[3]));
        //4,5 electrical output frequency
        //6,7 delta resolver filtered
        break;

    case 0x0A6:
        //0,1 Phase A current
        //2,3 Phase B current
        //4,5 Phase C current
        //6,7 DC bus current
        mcm->DC_Current = ((mcmCanMessage->data[6] << 8) | (mcmCanMessage->data[7]));
        break;

    case 0x0A7:
        //0,1 DC bus voltage***
        mcm->DC_Voltage = ((mcmCanMessage->data[0] << 8) | (mcmCanMessage->data[1]));
        //2,3 output voltage
        //4,5 Phase AB voltage
        //6,7 Phase BC voltage
        break;

    case 0x0A8:
        //0,1 Flux Command
        //2,3 flux feedback
        //4,5 id feedback
        //6,7 iq feedback
        break;

    case 0x0A9:
        // 0,1 1.5V reference voltage
        // 2,3 2.5V reference voltage
        // 4,5 5.0V reference voltage
        // 6,7 12V reference voltage
        break;

    case 0x0AA:

        //0,1 VSM state
        //2   Inverter state
        //3   Relay State
        //4   bit-0 inverter run mode
        //4   bit5-7 inverter active discharge state
        //5   inverter command mode
        //6   bit0 inverter enable state***
        mcm->inverterStatus = mcmCanMessage->data[6];
        //6   bit7 inverter enable lockout***
        mcm->lockoutStatus = mcmCanMessage->data[6];
        //7   direction command
        break;
    case 0x0AC:
        //0,1 Commanded Torque
        mcm->commandedTorque = ((ubyte2)mcmCanMessage->data[0] << 8) | (ubyte2)(mcmCanMessage->data[1]);
        //2,3 Torque Feedback
        break;

    }
}

MotorController* MotorController_new(ubyte2 canMessageBaseID, Direction initialDirection, ubyte2 torqueMaxInDNm)
{
	MotorController* me = (MotorController*)malloc(sizeof(struct _MotorController));

	me->canMessageBaseId = canMessageBaseID;
	//Dummy timestamp for last MCU message
	mcm_commands_resetUpdateCountAndTime(me);

	me->lockoutStatus = UNKNOWN;
	me->inverterStatus = UNKNOWN;
	//me->startRTDS = FALSE;

	me->commands_direction = initialDirection;
	me->torqueMaximum = torqueMaxInDNm;

	me->startupStage = 0; //Off

	/*
me->setTorque = &setTorque;
me->setInverter = &setInverter;
me->setDischarge = &setDischarge;
me->setTorqueLimit = &setTorqueLimit;
me->updateLockoutStatus = &updateLockoutStatus;
me->updateInverterStatus = &updateInverterStatus;
me->getLockoutStatus = &getLockoutStatus;
me->getInverterStatus = &getInverterStatus;
		*/
	return me;
}


//Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void mcm_commands_setTorque(MotorController* me, ubyte2 newTorque)
{
	me->updateCount += (me->commands_torque == newTorque) ? 0 : 1;
	me->commands_torque = newTorque;
}

void mcm_commands_setDirection(MotorController* me, Direction newDirection)
{
	switch (newDirection)
	{
	case _0:
	case CLOCKWISE:
	case REVERSE:
		me->updateCount += (me->commands_direction == 0) ? 0 : 1;
		me->commands_direction = 0;
		break;

	case _1:
	case COUNTERCLOCKWISE:
	case FORWARD:
		me->updateCount += (me->commands_direction == 1) ? 0 : 1;
		me->commands_direction = 1;
		break;

	default:
		//Invalid direction?
		break;
	}
}
void mcm_commands_setInverter(MotorController* me, Status newInverterState)
{
	me->updateCount += (me->commands_inverter == newInverterState) ? 0 : 1;
	me->commands_inverter = newInverterState;
}
void mcm_commands_setDischarge(MotorController* me, Status setDischargeTo)
{
	me->updateCount += (me->commands_discharge == setDischargeTo) ? 0 : 1;
	me->commands_discharge = setDischargeTo;
}
void mcm_commands_setTorqueLimit(MotorController* me, ubyte2 newTorqueLimit)
{
	me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
	me->commands_torqueLimit = newTorqueLimit;
}

ubyte2 mcm_command_getPower(MotorController* me)
{
	return (me->DC_Voltage * me->DC_Current);
}

ubyte2 mcm_command_getCommandedTorque(MotorController* me)
{
	return me->commandedTorque;
}

ubyte2 mcm_commands_getTorque(MotorController* me)
{
	return me->commands_torque;
}
Direction mcm_commands_getDirection(MotorController* me)
{
	return me->commands_direction;
}
Status mcm_commands_getInverter(MotorController* me)
{
	return me->commands_inverter;
}
Status mcm_commands_getDischarge(MotorController* me)
{
	return me->commands_discharge;
}
ubyte2 mcm_commands_getTorqueLimit(MotorController* me)
{
	return me->commands_torqueLimit;
}

void mcm_updateLockoutStatus(MotorController* me, Status newState)
{
	me->lockoutStatus = newState;
}
void mcm_updateInverterStatus(MotorController* me, Status newState)
{
	me->inverterStatus = newState;
}

Status mcm_getLockoutStatus(MotorController* me)
{
	return me->lockoutStatus;
}

Status mcm_getInverterStatus(MotorController* me)
{
	return me->inverterStatus;
}

void mcm_setRTDSFlag(MotorController* me, bool enableRTDS)
{
	//me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
	//me->startRTDS = enableRTDS;
}
bool mcm_getRTDSFlag(MotorController* me)
{
	//me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
	//return me->startRTDS;
	return FALSE;
}

ubyte2 mcm_commands_getUpdateCount(MotorController* me)
{
	return me->updateCount;
}

void mcm_commands_resetUpdateCountAndTime(MotorController* me)
{
	me->updateCount = 0;
	IO_RTC_StartTime(&(me->timeStamp_lastCommandSent));
}

ubyte4 mcm_commands_getTimeSinceLastCommandSent(MotorController* me)
{
	return IO_RTC_GetTimeUS(me->timeStamp_lastCommandSent);
}


ubyte2 mcm_getTorqueMax(MotorController* me)
{
	return me->torqueMaximum;
}


void mcm_setStartupStage(MotorController* me, ubyte1 stage)
{
	me->startupStage = stage;
}

ubyte1 mcm_getStartupStage(MotorController* me)
{
	return me->startupStage;
}



//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//Non-object-related functions
//----------------------------------------------------------------------------




/*****************************************************************************
	* Motor Control Functions
	* Reads sensor objects and sets MCM control object values, which will be picked up
	* later by CAN function
	* > Direction
	* > Torque
	*   > Delay command after inverter enable (temporary until noise fix)
	*   > Calculate Nm to request based on pedal position
	*   > Keep track of update count to prevent CANbus overload //MOVE ALL COUNT UPDATES INTO INTERNAL MCM FUNCTIONS
	* > Torque limit
	*   > Get from TCS function
	* Manages the different phases startup/ready-to-drive procedure
	* > Turn on MCM relay?  Should this be done elsewhere?
	* > Disable inverter lockout
	* > Listen for driver to complete startup sequence
	* > Enable inverter
	* > Play RTDS
	****************************************************************************/
void setMCMCommands(MotorController* mcm, TorqueEncoder* tps, BrakePressureSensor* bps, ReadyToDriveSound* rtds, SafetyChecker* sc)
{
	//----------------------------------------------------------------------------
	// Control commands
	//----------------------------------------------------------------------------
	//Temp hardcode
	mcm_commands_setDischarge(mcm, DISABLED);

	//1 = forwards for our car, 0 = reverse
	mcm_commands_setDirection(mcm, FORWARD);

	//Set Torque/Inverter control
	if (SafetyChecker_allSafe(sc) == FALSE)
	{
		mcm_commands_setTorque(mcm, 0);
	}
	else
	{
		mcm_commands_setTorque(mcm, mcm_getTorqueMax(mcm) * tps->percent);
		/*        ubyte2 torqueSetting;  //temp variable to store torque calculation
    //CURRENTLY: Don't command torque until >1s after the inverter is enabled, otherwise CAN breaks
    if (IO_RTC_GetTimeUS(mcm.timeStamp_inverterEnabled) <= 1000000)
    {
    torqueSetting = 0;
    if (mcm.commands.requestedTorque != torqueSetting) mcm.commands.updateCount++;
    }
    else
    {
    torqueSetting = 100 * getPercent(Sensor_WPS_FL.sensorValue, 500, 2500, TRUE); //int is fine
    if (mcm.commands.requestedTorque != torqueSetting) mcm.commands.updateCount++;
    mcm.commands.requestedTorque = torqueSetting;
    }
			*/
	}
}


//See diagram at https://onedrive.live.com/redir?resid=F9BB8F0F8FDB5CF8!30410&authkey=!ABSF-uVH-VxQRAs&ithint=file%2chtml
void MotorControllerPowerManagement(MotorController* mcm, TorqueEncoder* tps, ReadyToDriveSound* rtds)
{



	//If HVIL Term Sense is high, then set MCM relay high
	if (Sensor_HVILTerminationSense.sensorValue == FALSE)
	{
		setMCMRelay(FALSE);
		mcm_setStartupStage(mcm, 0);
	}
	else
	{
		//If the motor controller is off, don't turn it on until the pedals are calibrated
		//if (mcm_getStartupStage(mcm) == 0)
		//{
		setMCMRelay(TRUE);
		//mcm_setStartupStage(mcm, 1);
		//}
	}

	//----------------------------------------------------------------------------
	// Determine inverter state
	//----------------------------------------------------------------------------
	//New Handshake NOTE: Switches connected to ground.. TRUE = high = off = disconnected = open circuit, FALSE = low = grounded = on = connected = closed circuit
	//if (mcm_getLockoutStatus(mcm) == ENABLED)

	//Set inverter to disabled until RTD procedure is done.
	//This disables the lockout ahead of time.
	if (mcm_getStartupStage(mcm) < 3)
	{
		mcm_commands_setInverter(mcm, DISABLED);
		Light_set(Light_dashRTD, 0);
		//mcm_setStartupStage(mcm, 2);
	}

	//if (mcm_getStartupStage(mcm) == 1 && mcm_getRTDSFlag(mcm) == )
	//{
	//	mcm_commands_setInverter(mcm, DISABLED);
	//	mcm_setStartupStage(mcm, 2);
	//}

	//case DISABLED: //Lockout is disabled
	switch (mcm_getInverterStatus(mcm))
	{
	case DISABLED:
		mcm_setStartupStage(mcm, 3); //Lockout disabled, waiting for RTD procedure
		//If not on gas and YES on brake and RTD is pressed
		//BRAKE CODE NEEDS TO BE ADDED HERE
		if (tps->percent < .05 && Sensor_RTDButton.sensorValue == FALSE)
		{
			mcm_commands_setInverter(mcm, ENABLED);
			//mcm_setRTDSFlag(mcm, TRUE);  //Now, start the RTDS if the inverter is successfully enabled
			mcm_setStartupStage(mcm, 4); //RTD complete, waiting for confirmation
		}
		break;

	case ENABLED:
		Light_set(Light_dashRTD, 1);
		//If the inverter was successfully enabled AND we haven't started the RTDS yet
		//if (mcm_getRTDSFlag(mcm) == TRUE)
		if (mcm_getStartupStage(mcm) == 4) //If we're waiting to start the motor controller
		{
			RTDS_setVolume(rtds, .01, 1500000);
			//mcm_setRTDSFlag(mcm, FALSE);  //RTDS started, so don't restart it next loop
			mcm_setStartupStage(mcm, 5); //RTD confirmed
		}
		else
		{
			mcm_setStartupStage(mcm, 6); //Driving
		}
		break;

	case UNKNOWN: default:
		break;
	}

	//break;
	//case UNKNOWN: default:
	//    break;
	//}

	/*
//TEMPORARY Eco Switch startup code
if (Sensor_EcoButton.sensorValue == FALSE)
{
    mcm_commands_setInverter(mcm, DISABLED);
}
else
{
    mcm_commands_setInverter(mcm, ENABLED);
}
//If the inverter is disabled, but we're turning it on now
if (mcm_getInverterStatus(mcm) == DISABLED && mcm_commands_getInverter(mcm) == ENABLED)
{
    mcm_setRTDSFlag(mcm, TRUE);
}
if (mcm_getInverterStatus(mcm) == ENABLED && mcm_getRTDSFlag(mcm) == TRUE)
{
    RTDS_setVolume(rtds, .005, 1500000);
    mcm_setRTDSFlag(mcm, FALSE);  //RTDS started, so don't restart it next loop
}
		*/

}
