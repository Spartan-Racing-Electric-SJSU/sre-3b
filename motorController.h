//"Include guard" - prevents this file from being #included more than once
#ifndef _MOTORCONTROLLER_H
#define _MOTORCONTROLLER_H

#include "readyToDriveSound.h"

typedef enum { ENABLED, DISABLED } Status;

/*****************************************************************************
* Motor Controller (MCU)
******************************************************************************
*
****************************************************************************/

typedef struct _MCUCommand {
    //----------------------------------------------------------------------------
    // Control parameters
    //----------------------------------------------------------------------------
    // These are set by ??????????
    //----------------------------------------------------------------------------
    ubyte4 timeStamp_lastCommandSent;  //from IO_RTC_StartTime(&)
    ubyte2 updateCount; //Number of updates since lastCommandSent

    ubyte2 requestedTorque;
    ubyte2 requestedTorqueLimit;
    ubyte1 direction;

    //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
    Status setDischarge;
    Status setInverter;
    //bool inverterHasBeenEnableed;
    ubyte4 timeStamp_inverterEnabled;
    //ubyte1 controlSwitches; // example: 0b00000001 = inverter is enabled, discharge is disabled.

} MCUCommand;

typedef struct _MotorController {
    //----------------------------------------------------------------------------
    // Controller-specific properties
    //----------------------------------------------------------------------------
    // These should be identified at compile time (hard coded) for each motor
    // controller. (We may have more than 1 controller in the future)
    //----------------------------------------------------------------------------
    ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller
    
    //----------------------------------------------------------------------------
    // Controller statuses/properties
    //----------------------------------------------------------------------------
    // These represent the state of the controller (set at run time, not compile
    // time.)  These are updated by canInput.c
    //----------------------------------------------------------------------------
    Status lockoutStatus;
    Status inverterStatus;

    ubyte4 vsmStatus0;      //0xAA Byte 0,1
    ubyte4 vsmStatus1;      //0xAA Byte 0,1
    ubyte4 vsmStatus2;      //0xAA Byte 0,1
    ubyte4 vsmStatus3;      //0xAA Byte 0,1
    ubyte4 faultCodesPOST; //0xAB Byte 0-3
    ubyte4 faultCodesRUN;  //0xAB Byte 4-7
    
    //----------------------------------------------------------------------------
    // Control parameters
    //----------------------------------------------------------------------------
    // These are updated by ??? and will be sent to the VCU over CAN
    //----------------------------------------------------------------------------
    MCUCommand commands;

} MotorController;


//----------------------------------------------------------------------------
// Motor Controller Object Instantiations
//----------------------------------------------------------------------------
//CAN Message ID should be defined for each motor controller here.
extern MotorController MCU0; // = { 0xA0 };


//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------
//void motorController_UpdateFromCan(IO_CAN_DATA_FRAME *canMessage); //Update the MCU object from its CAN messages
//void motorController_SendControlMessage(IO_CAN_DATA_FRAME *canMessage); //This is an alias for canOutput_sendMcuControl
void motorController_setCommands(ReadyToDriveSound* rtds);

#endif // _MOTORCONTROLLER_H
