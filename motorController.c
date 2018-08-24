#include <stdlib.h>  //Needed for malloc
#include "IO_Driver.h"
#include "IO_DIO.h"     //TEMPORARY - until MCM relay control  / ADC stuff gets its own object
#include "IO_RTC.h"
#include "IO_CAN.h"

#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"
#include "sensorCalculations.h"

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"
#include "serial.h"

#include "canManager.h"


extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSSwitchUp; // used currently for regen
extern Sensor Sensor_TCSSwitchDown; // used currently for regen
extern Sensor Sensor_TCSKnob;    // used currently for regen
extern Sensor Sensor_HVILTerminationSense;

/*****************************************************************************
 * Motor Controller (MCM)
 ******************************************************************************
 *
 ****************************************************************************/

struct _MotorController {
    SerialManager* serialMan;
	//----------------------------------------------------------------------------
	// Controller statuses/properties
	//----------------------------------------------------------------------------
	// These represent the state of the controller (set at run time, not compile
	// time.)  These are updated by canInput.c
	//----------------------------------------------------------------------------
	ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller
	ubyte4 timeStamp_inverterEnabled;

    //Motor controller torque units are in 10ths (500 = 50.0 Nm)
    //Positive = accel, negative = regen
    //Reverse not allowed
	ubyte2 torqueMaximumDNm;  //Max torque that can be commanded in deciNewton*meters ("100" = 10.0 Nm)
	
	//Regen torque calculations in whole Nm..?
	ubyte1 regen_mode;					  //Software reading of regen knob position.  Each mode has different regen behavior (variables below).
	ubyte2 regen_torqueLimitDNm;          //Tuneable value.  Regen torque (in Nm) at full regen.  Positive value.
	ubyte2 regen_torqueAtZeroPedalDNm;    //Tuneable value.  Amount of regen torque (in Nm) to apply when both pedals at 0% travel.  Positive value.
	float4 regen_percentBPSForMaxRegen;   //Tuneable value.  Amount of brake pedal required for full regen. Value between zero and one.
	float4 regen_percentAPPSForCoasting;  //Tuneable value.  Amount of accel pedal required to exit regen.  Value between zero and one.
    sbyte1 regen_minimumSpeedKPH;  //Assigned by main
    sbyte1 regen_SpeedRampStart;

    bool relayState;
    bool previousHVILState;
    ubyte4 timeStamp_HVILLost;

    ubyte4 timeStamp_HVILOverrideCommandReceived;
    bool HVILOverride;

    ubyte1 startupStage;
    Status lockoutStatus;
	Status inverterStatus;
	bool startRTDS;
	/*ubyte4 vsmStatus0;      //0xAA Byte 0,1
    ubyte4 vsmStatus1;      //0xAA Byte 0,1
    ubyte4 vsmStatus2;      //0xAA Byte 0,1
    ubyte4 vsmStatus3;      //0xAA Byte 0,1
    ubyte4 faultCodesPOST; //0xAB Byte 0-3
    ubyte4 faultCodesRUN;  //0xAB Byte 4-7*/

    ubyte1 faultHistory[8];

	sbyte2 motor_temp;
	sbyte4 DC_Voltage;
	sbyte4 DC_Current;

	sbyte2 commandedTorque;
	ubyte4 currentPower;

	sbyte2 motorRPM;
	//----------------------------------------------------------------------------
	// Control parameters
	//----------------------------------------------------------------------------
	// These are updated by ??? and will be sent to the MCM over CAN
	//----------------------------------------------------------------------------
	//struct _commands {
	ubyte4 timeStamp_lastCommandSent;  //from IO_RTC_StartTime(&)
	ubyte2 updateCount; //Number of updates since lastCommandSent

	sbyte2 commands_torque;
	sbyte2 commands_torqueLimit;
	ubyte1 commands_direction;
	//unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
	Status commands_discharge;
	Status commands_inverter;
	//ubyte1 controlSwitches; // example: 0b00000001 = inverter is enabled, discharge is disabled
    
    /*
    //----------------------------------------------------------------------------
    // Control functions - for functions nested within struct
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
};

MotorController* MotorController_new(SerialManager* sm, ubyte2 canMessageBaseID, Direction initialDirection, sbyte2 torqueMaxInDNm, sbyte1 minRegenSpeedKPH, sbyte1 regenRampdownStartSpeed)
{
	MotorController* me = (MotorController*)malloc(sizeof(struct _MotorController));
    me->serialMan = sm;

	me->canMessageBaseId = canMessageBaseID;
	//Dummy timestamp for last MCU message
	MCM_commands_resetUpdateCountAndTime(me);

	me->lockoutStatus = UNKNOWN;
	me->inverterStatus = UNKNOWN;
	//me->startRTDS = FALSE;

    me->motorRPM = 0;
    me->DC_Voltage = 0;
    me->DC_Current = 0;

	me->commands_direction = initialDirection;
	me->commands_torqueLimit = me->torqueMaximumDNm = torqueMaxInDNm;

	me->regen_mode = 0xFF;
	me->regen_torqueLimitDNm = 0;
	me->regen_torqueAtZeroPedalDNm = 0;
    me->regen_percentBPSForMaxRegen = 1; //zero to one.. 1 = 100%
	me->regen_percentAPPSForCoasting = 0;
    me->regen_minimumSpeedKPH = minRegenSpeedKPH;  //Assigned by main
    me->regen_SpeedRampStart = regenRampdownStartSpeed;  //Assigned by main

    //me->faultHistory = { 0,0,0,0,0,0,0,0 };  //Todo: read from eeprom instead of defaulting to 0

	me->startupStage = 0; //Off
    
    me->relayState = FALSE; //Low

    me->motor_temp = 99;
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

//Note: As of 2017-01-27, TCS Switch has been removed from SRE-2.  Control is now selected based on 8-position knob (pot)
//This function sets all of the regen variables based on the knob's position (relative to 5 positions marked on dash)
//Knob Positions
//Pos  Hex   Dec  Comment
// 0   FFFF       Knob clicked off
// .      1    1
// 1      4    4
// .     A1  161
// 2    17E  382
// .    230  560
// 3    2FF  767
// .    383  899
// 4    3DA  986
// .    3DA  986

void MCM_readTCSSettings(MotorController* me, Sensor* TCSSwitchUp, Sensor* TCSSwitchDown, Sensor* TCSPot)
{	
	 

	//If the pot is clicked off (resistance goes to FFFF)
	if (TCSPot->sensorValue > 5000)  //Position 0 = Regen Off
    {
		me->regen_mode = 0;  

		me->regen_torqueLimitDNm = 0;
		me->regen_torqueAtZeroPedalDNm = 0;
		me->regen_percentAPPSForCoasting = 0;
		me->regen_percentBPSForMaxRegen = 0; //zero to one.. 1 = 100%
	}
	else if (TCSPot->sensorValue < 0xA1)  //Position 1 = Coasting mode (Formula E mode)
	{
		me->regen_mode = 1;
		me->regen_torqueLimitDNm = me->torqueMaximumDNm * 0.5;
		me->regen_torqueAtZeroPedalDNm = 0;
		me->regen_percentAPPSForCoasting = 0;
		me->regen_percentBPSForMaxRegen = .3; //zero to one.. 1 = 100%
	}
	else if (TCSPot->sensorValue < 0x230)  //Position 2 = light "engine braking" (Hybrid mode)
	{
		me->regen_mode = 2;
		me->regen_torqueLimitDNm = me->torqueMaximumDNm * 0.5;
		me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm * 0.3;
		me->regen_percentAPPSForCoasting = .2;
		me->regen_percentBPSForMaxRegen = .3; //zero to one.. 1 = 100%
	}
	else if (TCSPot->sensorValue < 0x383)  //Position 3 = One pedal driving (Tesla mode)
	{
		me->regen_mode = 3;
		me->regen_torqueLimitDNm = me->torqueMaximumDNm * 0.5;
		me->regen_torqueAtZeroPedalDNm = me->regen_torqueLimitDNm;
		me->regen_percentAPPSForCoasting = .1;
		me->regen_percentBPSForMaxRegen = 0;
	}
	else if (TCSPot->sensorValue >= 0x383)  //Position 4 = User customizable
	{
		me->regen_mode = 4;
		me->regen_torqueLimitDNm = 0;
		me->regen_torqueAtZeroPedalDNm = 0;
		me->regen_percentBPSForMaxRegen = 0; //zero to one.. 1 = 100%
		me->regen_percentAPPSForCoasting = 0;
	}
	else  //This should never happen
	{
		me->regen_mode = 0xFFFF;  //Default: Regen off
		me->regen_torqueLimitDNm = 0;
		me->regen_torqueAtZeroPedalDNm = 0;
		me->regen_percentBPSForMaxRegen = 0; //zero to one.. 1 = 100%
		me->regen_percentAPPSForCoasting = 0;
	}
}

/*****************************************************************************
* Motor Control Functions
* Reads sensor objects and sets MCM control object values, which will be picked up
* later by CAN function
* > Direction
* > Torque
*   - Delay command after inverter enable (temporary until noise fix)
*   - Calculate Nm to request based on pedal position
*   - DO NOT limit Nm based on external system limits - that is handled by safety.c,
*     and we still want to know the driver's requested torque
*   - DO limit Nm based on driver inputs (dash settings for regen)
*   - Keep track of update count to prevent CANbus overload
* > Torque limit
*   - Get from TCS function
* Manages the different phases startup/ready-to-drive procedure
* > Turn on MCM relay?  Should this be done elsewhere?
* > Disable inverter lockout
* > Listen for driver to complete startup sequence
* > Enable inverter
* > Play RTDS
****************************************************************************/
void MCM_calculateCommands(MotorController* me, TorqueEncoder* tps, BrakePressureSensor* bps)
{
	//----------------------------------------------------------------------------
	// Control commands
    //Note: Safety checks (torque command limiting) are done EXTERNALLY.  This is a preliminary calculation
    //which should return the intended torque based on pedals
    //Note: All stored torque values should be positive / unsigned
    //----------------------------------------------------------------------------
	MCM_commands_setDischarge(me, DISABLED);
	MCM_commands_setDirection(me, FORWARD); //1 = forwards for our car, 0 = reverse
    
	sbyte2 torqueOutput = 0;
	sbyte2 appsTorque = 0;
	sbyte2 bpsTorque = 0;
    // temporary change
    // if (me->torqueMaximumDNm > 50) me->torqueMaximumDNm = 50;
	appsTorque = me->torqueMaximumDNm * getPercent(tps->percent, me->regen_percentAPPSForCoasting, 1, TRUE) - me->regen_torqueAtZeroPedalDNm * getPercent(tps->percent, me->regen_percentAPPSForCoasting, 0, TRUE);
	bpsTorque = 0 - (me->regen_torqueLimitDNm - me->regen_torqueAtZeroPedalDNm) * getPercent(bps->percent, 0, me->regen_percentBPSForMaxRegen, TRUE);
	
	torqueOutput = appsTorque + bpsTorque;
    //torqueOutput = me->torqueMaximumDNm * tps->percent;  //REMOVE THIS LINE TO ENABLE REGEN
    MCM_commands_setTorqueDNm(me, torqueOutput);

    me->HVILOverride = (IO_RTC_GetTimeUS(me->timeStamp_HVILOverrideCommandReceived) < 1000000);
}

void MCM_relayControl(MotorController* me, Sensor* HVILTermSense)
{    
    //If HVIL Term Sense is low (HV is down)
    if (HVILTermSense->sensorValue == FALSE && me->HVILOverride == FALSE)
    {
        //If we just noticed the HVIL went low
        if (me->previousHVILState == TRUE)
        {
            SerialManager_send(me->serialMan, "Term sense went low\n");
            IO_RTC_StartTime(&me->timeStamp_HVILLost); //start counting time
        }

        //If the MCM is on (and we lost HV)
        if (me->relayState == TRUE)
        {
            //Okay to turn MCM off once 0 torque is commanded, or after 2 sec from losing HVIL
            //TODO: SIMILAR CODE SHOULD BE EMPLOYED AT HVIL SHUTDOWN CONTROL PIN
            if (me->commandedTorque == 0 || IO_RTC_GetTimeUS(me->timeStamp_HVILLost) > 2000000)  //2 s = 2,000 ms = 2,000,000 us
            {
                IO_DO_Set(IO_DO_00, FALSE);  //turn off MCM relay at IO_PIN_144 //(Rusty)Need MCM relay object
                me->relayState = FALSE;
            }
            else
            {
                //Safety.c will command zero torque
                //For now do nothing
            }
        }
        MCM_setStartupStage(me, 0);
        MCM_updateInverterStatus(me, UNKNOWN);
        MCM_updateLockoutStatus(me, UNKNOWN);

        me->previousHVILState = FALSE;
    }
    else  // HVILTermSense->sensorValue == TRUE || me->HVILOverride == TRUE
    {
        //If HVIL just changed, send a message
        if (me->previousHVILState == FALSE)
        {
            SerialManager_send(me->serialMan, "Term sense went high\n");
            if (MCM_getStartupStage(me) == 0)
            {
                MCM_setStartupStage(me, 1); //Reset the startup procedure because HV just went high and we are now turning on the MCM
            }
        }
        me->previousHVILState = TRUE;

        //Turn on the MCM relay
        IO_DO_Set(IO_DO_00, TRUE); //IO_PIN_144
        me->relayState = TRUE;
    }
}

//See diagram at https://onedrive.live.com/redir?resid=F9BB8F0F8FDB5CF8!30410&authkey=!ABSF-uVH-VxQRAs&ithint=file%2chtml
void MCM_inverterControl(MotorController* me, TorqueEncoder* tps, BrakePressureSensor* bps, ReadyToDriveSound* rtds)
{
    float4 RTDPercent = 0;
    RTDPercent = (Sensor_RTDButton.sensorValue == TRUE ? 1 : 0);
    
	//----------------------------------------------------------------------------
	// Determine inverter state
	//----------------------------------------------------------------------------
	//New Handshake NOTE: Switches connected to ground.. TRUE = high = off = disconnected = open circuit, FALSE = low = grounded = on = connected = closed circuit
    switch (MCM_getStartupStage(me))
    {
    case 0: //MCM relay is off --> stay until lockout is enabled;
        //It shouldn't be necessary to check mcm relayState or inverter status <> UNKNOWN
        //but if we have trouble we can add that here.

    case 1: //MCM relay is on, lockout=enabled, inverter=disabled --> stay until lockout is disabled
        //Actions to perform upon entering this state ------------------------------------------------
        MCM_commands_setInverter(me, DISABLED);
        //Light_set(Light_dashRTD, 0);

        //How to transition to next state ------------------------------------------------
        if (MCM_getLockoutStatus(me) == DISABLED)
        {
            SerialManager_send(me->serialMan, "MCM lockout has been disabled.\n");
            MCM_setStartupStage(me, 2); //MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);
        }
        break;

    case 2: //MCM on, lockout=disabled, inverter=disabled --> stay until RTD button pressed
        //Actions to perform upon entering this state ------------------------------------------------
        //Nothing: wait for RTD button

        //How to transition to next state ------------------------------------------------
        if (Sensor_RTDButton.sensorValue == TRUE // Could just put RTDPercent
            && tps->calibrated == TRUE
            && bps->calibrated == TRUE
            && tps->percent < .1
            && bps->percent > .25
            )
        {
            MCM_commands_setInverter(me, ENABLED);  //Change the inverter command to enable
            SerialManager_send(me->serialMan, "Changed MCM inverter command to ENABLE.\n");
            MCM_setStartupStage(me, 3); // MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);
            //RTDPercent = 1; //no RTD light at this stage, we dont know if inverter is really enabled yet. It will be checked on the next stage
        }
        break;

    case 3: //inverted=disabled, rtd=pressed, waiting for inverter to be enabled
        //Actions to perform upon entering this state ------------------------------------------------
        //Nothing: wait for mcm to say it's enabled

        //How to transition to next state ------------------------------------------------
        if (MCM_getInverterStatus(me) == ENABLED)
        {
            RTDPercent = 1; //Doesn't matter if button is no longer pressed - RTD light should be on if car is driveable
            SerialManager_send(me->serialMan, "Inverter has been enabled.  Starting RTDS.  Car is ready to drive.\n");
            RTDS_setVolume(rtds, 0, 1500000); // value 0 at normal testing (and 1 for the real ones, and of course at the comp), you dont want an eardrum rupture everytime right?
            MCM_setStartupStage(me, 4); //MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);  //leave this stage since we've already kicked off the RTDS
        }
        break;

    case 4: //inverter=enabled, rtds=already started
        //Actions to perform upon entering this state ------------------------------------------------
        //If anything happened disabling inverter, MCM_relayControl(), the function right above this one, will catch it before this MCM_inverterControl() function and throw the state back to stage 0.
        SerialManager_send(me->serialMan, "RTD procedure complete.\n");  //Just send a message
        RTDPercent = 1; //RTD light should be on if car is driveable

        //How to transition to next state ------------------------------------------------
        //Always do, since we sent a message.
        MCM_setStartupStage(me, 5); //MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);
        break;

    case 5: //inverter=enabled, rtds=already started, no safety fault occured
        //What happens in this state ------------------------------------------------
        //The car is safe to drive in this stage
        RTDPercent = 1; //RTD light should be on if car is driveable
        //This case is here so we don't send a message anymore
        
        //How to transition to next state ------------------------------------------------
        //Don't. //no more MCM_setStartupStage(me, MCM_getStartupStage(me) + 1);
        break;


    default:
        SerialManager_send(me->serialMan, "ERROR: Lost track of MCM startup status.\n");
        break;
    }
    
    //After all that, we can turn the RTD light on/off
    Light_set(Light_dashRTD, RTDPercent);

}


void MCM_parseCanMessage(MotorController* me, IO_CAN_DATA_FRAME* mcmCanMessage)
{
    //0xAA
    static const ubyte1 bitInverter = 1; //bit 0, 0000 0001
    static const ubyte1 bitLockout = 128; //bit 7, 1000 0000

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
        me->motor_temp = ((ubyte2)mcmCanMessage->data[5] << 8 | mcmCanMessage->data[4]) / 10;
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
        //Cast may be required - needs testing
        me->motorRPM = (ubyte2)mcmCanMessage->data[3] << 8 | mcmCanMessage->data[2];
        //me->motorRPM = ((mcmCanMessage->data[2] << 8) | (mcmCanMessage->data[3]));
        //4,5 electrical output frequency
        //6,7 delta resolver filtered
        break;

    case 0x0A6:
        //0,1 Phase A current
        //2,3 Phase B current
        //4,5 Phase C current
        //6,7 DC bus current
        me->DC_Current = ((ubyte2)mcmCanMessage->data[7] << 8 | mcmCanMessage->data[6]) / 10;
        //me->DC_Current = (((mcmCanMessage->data[6] << 8) | (mcmCanMessage->data[7])) / 10);
        break;

    case 0x0A7:
        //0,1 DC bus voltage***
        me->DC_Voltage = ((ubyte2)mcmCanMessage->data[1] << 8 | mcmCanMessage->data[0]) / 10;
        //me->DC_Voltage = (((mcmCanMessage->data[0] << 8) | (mcmCanMessage->data[1])) / 10);
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

        //6   internal states
        //    bit0 inverter enable state***
        me->inverterStatus = (mcmCanMessage->data[6] & bitInverter) > 0 ? ENABLED : DISABLED;
        //    bit7 inverter enable lockout***
        me->lockoutStatus = (mcmCanMessage->data[6] & bitLockout) > 0 ? ENABLED : DISABLED;

        //7   direction command
        break;


    case 0x0AB: //Faults
        //mcmCanMessage->data;
        //me->faultHistory |= data stuff //????????

        break;


    case 0x0AC:
        //0,1 Commanded Torque
        me->commandedTorque = ((ubyte2)mcmCanMessage->data[1] << 8 | mcmCanMessage->data[0]) / 10;
        //2,3 Torque Feedback
        break;


    case 0x5FF:
        //0,1 Commanded Torque
        if (mcmCanMessage->data[1] > 0)
        {
            IO_RTC_StartTime(&me->timeStamp_HVILOverrideCommandReceived);
        }
        //2,3 Torque Feedback
        break;

    }
}


/*****************************************************************************
* Accessors / Mutators (Set/Get)
******************************************************************************
*
****************************************************************************/
//Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void MCM_commands_setTorqueDNm(MotorController* me, sbyte2 newTorque)
{
	me->updateCount += (me->commands_torque == newTorque) ? 0 : 1;
	me->commands_torque = newTorque;
}

void MCM_commands_setDirection(MotorController* me, Direction newDirection)
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
void MCM_commands_setInverter(MotorController* me, Status newInverterState)
{
	me->updateCount += (me->commands_inverter == newInverterState) ? 0 : 1;
	me->commands_inverter = newInverterState;
}
void MCM_commands_setDischarge(MotorController* me, Status setDischargeTo)
{
	me->updateCount += (me->commands_discharge == setDischargeTo) ? 0 : 1;
	me->commands_discharge = setDischargeTo;
}
void MCM_commands_setTorqueLimit(MotorController* me, sbyte2 newTorqueLimit)
{
	me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
	me->commands_torqueLimit = newTorqueLimit;
}
sbyte2 MCM_commands_getTorque(MotorController* me)
{
	return me->commands_torque;
}
Direction MCM_commands_getDirection(MotorController* me)
{
	return me->commands_direction;
}
Status MCM_commands_getInverter(MotorController* me)
{
	return me->commands_inverter;
}
Status MCM_commands_getDischarge(MotorController* me)
{
	return me->commands_discharge;
}
sbyte2 MCM_commands_getTorqueLimit(MotorController* me)
{
	return me->commands_torqueLimit;
}

void MCM_updateLockoutStatus(MotorController* me, Status newState)
{
	me->lockoutStatus = newState;
}
void MCM_updateInverterStatus(MotorController* me, Status newState)
{
	me->inverterStatus = newState;
}

Status MCM_getLockoutStatus(MotorController* me)
{
	return me->lockoutStatus;
}

Status MCM_getInverterStatus(MotorController* me)
{
	return me->inverterStatus;
}

bool MCM_getHvilOverrideStatus(MotorController* me)
{
    return me->HVILOverride;
}

void MCM_setRTDSFlag(MotorController* me, bool enableRTDS)
{
	me->startRTDS = enableRTDS;
}
bool MCM_getRTDSFlag(MotorController* me)
{
	//me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
	return me->startRTDS;
	//return FALSE;
}

ubyte2 MCM_commands_getUpdateCount(MotorController* me)
{
	return me->updateCount;
}

void MCM_commands_resetUpdateCountAndTime(MotorController* me)
{
	me->updateCount = 0;
	IO_RTC_StartTime(&(me->timeStamp_lastCommandSent));
}

ubyte4 MCM_commands_getTimeSinceLastCommandSent(MotorController* me)
{
	return IO_RTC_GetTimeUS(me->timeStamp_lastCommandSent);
}


ubyte2 MCM_getTorqueMax(MotorController* me)
{
	return me->torqueMaximumDNm;
}



sbyte4 MCM_getPower(MotorController* me)
{
	return ((me->DC_Voltage) * (me->DC_Current));
}

ubyte2 MCM_getCommandedTorque(MotorController* me)
{
	return me->commandedTorque;
}


sbyte2 MCM_getTemp(MotorController* me)
{
    return me->motor_temp;  //TODO: Figure out which temperature to return for Motor Controller
}
sbyte2 MCM_getMotorTemp(MotorController* me)
{
    return me->motor_temp;
}

sbyte2 MCM_getGroundSpeedKPH(MotorController* me)
{
    sbyte2 wheelRPM = me->motorRPM / 3;
    float4 tireCircumference = 3.141592653589 * 18 * .0254; // (pi * diameter * in to m) = circumference in meters
    sbyte2 groundKPH = wheelRPM / 60 * tireCircumference;
    return groundKPH;
}

ubyte1 MCM_getRegenMode(MotorController* me)
{
	return me->regen_mode;
}
sbyte2 MCM_getRegenTorqueLimitDNm(MotorController* me)
{
	return 0xFF * me->regen_torqueLimitDNm;
}
sbyte2 MCM_getRegenTorqueAtZeroPedalDNm(MotorController* me)
{
	return me->regen_torqueAtZeroPedalDNm;
}
sbyte2 MCM_getRegenBPSForMaxRegenZeroToFF(MotorController* me)
{
	return 0xFF * me->regen_percentBPSForMaxRegen;
}
sbyte2 MCM_getRegenAPPSForMaxCoastingZeroToFF(MotorController* me)
{
	return 0xFF * me->regen_percentAPPSForCoasting;
}

sbyte1 MCM_getRegenMinSpeed(MotorController* me)
{
    return me->regen_minimumSpeedKPH;
}
sbyte1 MCM_getRegenRampdownStartSpeed(MotorController* me)
{
    return me->regen_SpeedRampStart;
}


void MCM_setStartupStage(MotorController* me, ubyte1 stage)
{
	me->startupStage = stage;
}

ubyte1 MCM_getStartupStage(MotorController* me)
{
	return me->startupStage;
}
