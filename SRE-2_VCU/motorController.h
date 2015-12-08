//"Include guard" - prevents this file from being #included more than once
#ifndef _MOTORCONTROLLER_H
#define _MOTORCONTROLLER_H

//----------------------------------------------------------------------------
// Sensor Object Definitions
//----------------------------------------------------------------------------
// Parameters:
//
// specMin/Max values should come from each sensor's datasheets, but it is not
// required for all sensors.
//
// TODO: What about having default calbiration values?  (Probably useless)
//----------------------------------------------------------------------------
typedef struct _MotorController {
    //----------------------------------------------------------------------------
    // Controller-specific properties
    //----------------------------------------------------------------------------
    // These should be identified at compile time (hard coded) for each motor
    // controller.
    //----------------------------------------------------------------------------
    ubyte2 canMessageBaseId;  //Starting message ID for messages that will come in from this controller
        
    //----------------------------------------------------------------------------
    // Controller statuses/properties
    //----------------------------------------------------------------------------
    // These represent the state of the controller (set at run time, not compile
    // time.)
    //----------------------------------------------------------------------------
    bool lockoutDisabled;
    bool inverterEnabled;

    ubyte2 vsmStatus;      //0xAA Byte 0,1
    ubyte4 faultCodesPOST; //0xAB Byte 0-3
    ubyte4 faultCodesRUN;  //0xAB Byte 4-7


} MotorController;

//----------------------------------------------------------------------------
// Motor Controller Object Instantiations
//----------------------------------------------------------------------------
//CAN Message ID should be defined for each motor controller here.
MotorController MCU0 = { 0x0A0 };


//----------------------------------------------------------------------------
// Sensor Functions
//----------------------------------------------------------------------------
void sensors_updateSensors(void);

#endif // _MOTORCONTROLLER_H
