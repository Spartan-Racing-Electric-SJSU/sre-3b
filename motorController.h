//"Include guard" - prevents this file from being #included more than once
#ifndef _MOTORCONTROLLER_H
#define _MOTORCONTROLLER_H

#include "IO_Driver.h"
#include "readyToDriveSound.h"

typedef enum { ENABLED, DISABLED, UNKNOWN } Status;

//Rotation direction as viewed from shaft end of motor
//0 = CW = REVERSE (for our car)
//1 = CCW = FORWARD (for our car)
typedef enum { CLOCKWISE, COUNTERCLOCKWISE, FORWARD, REVERSE, _0, _1 } Direction;

typedef struct _MotorController MotorController;

MotorController* MotorController_new(ubyte2 canMessageBaseID, Direction initialDirection);

//----------------------------------------------------------------------------
// Functions
//----------------------------------------------------------------------------
//CAN Message Parameters
//Note: Speed Command (angular velocity) not used when in torque mode
void motorController_setTorque(MotorController* me, ubyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void motorController_setDirection(MotorController* me, Direction rotation);
void motorController_setInverter(MotorController* me, Status inverterState);
void motorController_setDischarge(MotorController* me, Status dischargeState);
void motorController_setTorqueLimit(MotorController* me, ubyte2 torqueLimit);

//void motorController_UpdateFromCan(IO_CAN_DATA_FRAME *canMessage); //Update the MCU object from its CAN messages
//void motorController_SendControlMessage(IO_CAN_DATA_FRAME *canMessage); //This is an alias for canOutput_sendMcuControl
void motorController_setAllCommands(ReadyToDriveSound* rtds);

#endif // _MOTORCONTROLLER_H
