#include <stdlib.h>  //Needed for malloc
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"
#include "readyToDriveSound.h"

/*****************************************************************************
* Motor Controller (MCU)
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

    Status lockoutStatus;
    Status inverterStatus;
    bool startRTDS;
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

        //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
        Status commands_discharge;
        Status commands_inverter;
        //ubyte1 controlSwitches; // example: 0b00000001 = inverter is enabled, discharge is disabled.

    //};

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
};


MotorController* MotorController_new(ubyte2 newCanMessageBaseID, Direction initialDirection)
{
    MotorController* me = (MotorController*)malloc(sizeof(struct _MotorController));

    me->canMessageBaseId = newCanMessageBaseID;
    //Dummy timestamp for last MCU message
    mcm_commands_resetUpdateCountAndTime(me);

    me->lockoutStatus = UNKNOWN;
    me->inverterStatus = UNKNOWN;
    me->startRTDS = FALSE;

/*
    me->setTorque = &setTorque;
    me->setDirection = &setDirection;
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
    me->startRTDS = enableRTDS;
}
bool mcm_getRTDSFlag(MotorController* me)
{
    //me->updateCount += (me->commands_torqueLimit == newTorqueLimit) ? 0 : 1;
    return me->startRTDS;
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
void setMCMCommands(MotorController* mcm, ReadyToDriveSound* rtds)
{
    //----------------------------------------------------------------------------
    // Control commands
    //----------------------------------------------------------------------------
    //Temp hardcode
    mcm_commands_setDischarge(mcm, DISABLED);

    //1 = forwards for our car, 0 = reverse
    mcm_commands_setDirection(mcm, FORWARD);

    //Set Torque/Inverter control
    if (Sensor_WPS_FL.fresh == FALSE)
    {
        mcm_commands_setTorque(mcm, 0);
    }
    else
    {
        mcm_commands_setTorque(mcm, 100 * getPercent(Sensor_WPS_FL.sensorValue, 500, 2500, TRUE));
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
void MotorControllerPowerManagement(MotorController* mcm, ReadyToDriveSound* rtds)
{
    //----------------------------------------------------------------------------
    // Determine inverter state
    //----------------------------------------------------------------------------
    //New Handshake NOTE: Switches connected to ground.. TRUE = high = off = disconnected = open circuit, FALSE = low = grounded = on = connected = closed circuit
    if (mcm_getLockoutStatus(mcm) == ENABLED)
    {
        mcm_commands_setInverter(mcm, DISABLED);
    }
    else  //Lockout has already been disabled
    {
        if (mcm_getInverterStatus(mcm) == DISABLED)
        {
            //If not on gas and YES on break and RTD is pressed
            if (Sensor_WPS_FL.sensorValue < 10 && Sensor_TEMP_BrakingSwitch.sensorValue == FALSE && Sensor_RTD_Button.sensorValue == FALSE)
            {
                mcm_commands_setInverter(mcm, ENABLED);
                mcm_setRTDSFlag(mcm, TRUE);  //Now, start the RTDS if the inverter is successfully enabled
            }
        }
        else
        {
            //If the inverter was successfully enabled AND we haven't started the RTDS yet
            if (mcm_getRTDSFlag(mcm) == TRUE)
            {
                RTDS_setVolume(rtds, .005, 1500000);
                mcm_setRTDSFlag(mcm, FALSE);  //RTDS started, so don't restart it next loop
            }
        }
    }
/*
    //TEMPORARY Eco Switch startup code
    if (Sensor_TEMP_BrakingSwitch.sensorValue == FALSE)
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