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


        //----------------------------------------------------------------------------
        // Control functions
        //----------------------------------------------------------------------------
        void(*motorController_setTorque)(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
        void(*motorController_setDirection)(MotorController* me, Direction rotation);
        void(*motorController_setInverter)(MotorController* me, Status inverterState);
        void(*motorController_setDischarge)(MotorController* me, Status dischargeState);
        void(*motorController_setTorqueLimit)(MotorController* me, ubyte2 torqueLimit);
//        void(*motorController_setTorque)(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
//        void(*motorController_setDirection)(MotorController* me, Direction rotation);
//        void(*motorController_setInverter)(MotorController* me, Status inverterState);
//        void(*motorController_setDischarge)(MotorController* me, Status dischargeState);
//        void(*motorController_setTorqueLimit)(MotorController* me, ubyte2 torqueLimit);
    //};
    //_commands commands;
};


MotorController* MotorController_new(ubyte2 canMessageBaseID, Direction initialDirection)
{
    MotorController* me = (MotorController*)malloc(sizeof(struct _MotorController));
    
    return me;
}


//Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void motorController_setTorque(MotorController* me, ubyte2 requestedTorque)
{
    me->commands_torque = requestedTorque;
}

void motorController_setDirection(MotorController* me, Direction rotation)
{
    switch (rotation)
    {
    case _0:
    case CLOCKWISE:
    case REVERSE:
        me->commands_direction = 0;
        break;

    case _1:
    case COUNTERCLOCKWISE:
    case FORWARD:
        me->commands_direction = 1;
        break;

    default:
        //Invalid direction?
        break;
    }
}
void motorController_setInverter(MotorController* me, Status inverter)
{
    me->commands_inverter = inverter;
}
void motorController_setDischarge(MotorController* me, Status discharge)
{
    me->commands_discharge = discharge;
}
void motorController_setTorqueLimit(MotorController* me, ubyte2 torqueLimit)
{
    me->commands_torqueLimit = torqueLimit;
}

/*
//Update the MCU object from its CAN messages
void motorController_setCommands(ReadyToDriveSound* rtds) 
{
    //Temp hardcode
    MCU0.commands.setDischarge = DISABLED;

    //1 = forwards for our car, 0 = reverse
    MCU0.commands.direction = 1;

    //Set Torque/Inverter control
    if (Sensor_WPS_FL.fresh == FALSE)
    {
        if (MCU0.commands.requestedTorque != 0) MCU0.commands.updateCount++;
        MCU0.commands.requestedTorque = 0;
    }
    else
    {
        ubyte2 torqueSetting;  //temp variable to store torque calculation
        //CURRENTLY: Don't command torque until >1s after the inverter is enabled, otherwise CAN breaks
        if (IO_RTC_GetTimeUS(MCU0.timeStamp_inverterEnabled) <= 1000000)
        {
            torqueSetting = 0;
            if (MCU0.commands.requestedTorque != torqueSetting) MCU0.commands.updateCount++;
        }
        else
        {
            torqueSetting = 100 * getPercent(Sensor_WPS_FL.sensorValue, 500, 2500, TRUE); //int is fine
            if (MCU0.commands.requestedTorque != torqueSetting) MCU0.commands.updateCount++;
            MCU0.commands.requestedTorque = torqueSetting;
        }
    }

    //New Handshake NOTE: Switches connected to ground.. TRUE = high = off = disconnected = open circuit, FALSE = low = grounded = on = connected = closed circuit
    if (MCU0.lockoutStatus == ENABLED)
    {
        //If not on gas and YES on break and RTD is pressed
        if (Sensor_WPS_FL.sensorValue < 10 && Sensor_TEMP_BrakingSwitch.sensorValue == FALSE && Sensor_RTD_Button.sensorValue == FALSE)
        {
            //Begin the startup sequence, but don't start the RTDS yet
            MCU0.commands.setInverter = DISABLED;
            MCU0.startRTDS = FALSE;
        }
    }
    else  //Lockout has already been disabled
    {
        if (MCU0.inverterStatus == DISABLED)
        {
            MCU0.commands.setInverter = ENABLED;
            IO_RTC_StartTime(&MCU0.timeStamp_inverterEnabled);
            MCU0.startRTDS = TRUE;
        }
        else  
        {
            //If the inverter was successfully enabled AND we haven't started the RTDS yet
            if (MCU0.startRTDS == TRUE)
            {
                RTDS_setVolume(rtds, .005, 1500000);
                MCU0.startRTDS = FALSE;                
            }
        }
    }
}
*/