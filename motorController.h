//"Include guard" - prevents this file from being #included more than once
#ifndef _MOTORCONTROLLER_H
#define _MOTORCONTROLLER_H

#include "IO_CAN.h"
#include "IO_Driver.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "readyToDriveSound.h"
//#include "safety.h"
#include "serial.h"

//typedef enum { TORQUE, DIRECTION, INVERTER, DISCHARGE, TORQUELIMIT} MCMCommand;
typedef enum { ENABLED, DISABLED, UNKNOWN } Status;

//Rotation direction as viewed from shaft end of motor
//0 = CW = REVERSE (for our car)
//1 = CCW = FORWARD (for our car)
typedef enum { CLOCKWISE, COUNTERCLOCKWISE, FORWARD, REVERSE, _0, _1 } Direction;

typedef struct _MotorController MotorController;

MotorController* MotorController_new(SerialManager* sm, ubyte2 canMessageBaseID, Direction initialDirection, sbyte2 torqueMaxInDNm, sbyte1 minRegenSpeedKPH, sbyte1 regenRampdownStartSpeed);

//----------------------------------------------------------------------------
// Command Functions
//----------------------------------------------------------------------------
//CAN Message Parameters
//Note: Speed Command (angular velocity) not used when in torque mode
void MCM_commands_setTorqueDNm(MotorController* me, sbyte2 torque); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
void MCM_commands_setDirection(MotorController* me, Direction rotation);
void MCM_commands_setInverter(MotorController* me, Status inverterState);
void MCM_commands_setDischarge(MotorController* me, Status dischargeState);
void MCM_commands_setTorqueLimit(MotorController* me, sbyte2 torqueLimit);
//void setCommand(MotorController* me, MCMCommand command, void* setting);


sbyte2 MCM_commands_getTorque(MotorController* me); //Will be divided by 10 e.g. pass in 100 for 10.0 Nm
Direction MCM_commands_getDirection(MotorController* me);
Status MCM_commands_getInverter(MotorController* me);
Status MCM_commands_getDischarge(MotorController* me);
sbyte2 MCM_commands_getTorqueLimit(MotorController* me); 

ubyte2 MCM_commands_getUpdateCount(MotorController* me);
void MCM_commands_resetUpdateCountAndTime(MotorController* me);
ubyte4 MCM_commands_getTimeSinceLastCommandSent(MotorController* me);

//----------------------------------------------------------------------------
// Update Functions (CAN Inputs)
//----------------------------------------------------------------------------
//void updatefromCAN(MotorController* me, CANFRAME or MOVE THIS EXTERNAL);
void MCM_updateLockoutStatus(MotorController* me, Status newState);
void MCM_updateInverterStatus(MotorController* me, Status newState);

//----------------------------------------------------------------------------
// Status Functions (CAN Inputs)
//----------------------------------------------------------------------------
Status MCM_getLockoutStatus(MotorController* me);
Status MCM_getInverterStatus(MotorController* me);

sbyte4 MCM_getPower(MotorController* me);
ubyte2 MCM_getCommandedTorque(MotorController* me);

bool MCM_getHvilOverrideStatus(MotorController* me);

void MCM_setRTDSFlag(MotorController* me, bool start);
bool MCM_getRTDSFlag(MotorController* me);
//MOVE ALL COUNT UPDATES INTO THESE FUNCTIONS

//void motorController_UpdateFromCan(IO_CAN_DATA_FRAME *canMessage); //Update the MCU object from its CAN messages
//void motorController_SendControlMessage(IO_CAN_DATA_FRAME *canMessage); //This is an alias for canOutput_sendMcuControl
//void motorController_setAllCommands(ReadyToDriveSound* rtds);

sbyte2 MCM_getTemp(MotorController* me);
sbyte2 MCM_getMotorTemp(MotorController* me);

sbyte2 MCM_getGroundSpeedKPH(MotorController* me);
sbyte1 MCM_getRegenMinSpeed(MotorController* me);
sbyte1 MCM_getRegenRampdownStartSpeed(MotorController* me);

ubyte1 MCM_getRegenMode(MotorController* me);
sbyte2 MCM_getRegenTorqueLimitDNm(MotorController* me);
sbyte2 MCM_getRegenTorqueAtZeroPedalDNm(MotorController* me);
sbyte2 MCM_getRegenBPSForMaxRegenZeroToFF(MotorController* me);
sbyte2 MCM_getRegenAPPSForMaxCoastingZeroToFF(MotorController* me);

//----------------------------------------------------------------------------
//Inter-object functions
//----------------------------------------------------------------------------
void MCM_readTCSSettings(MotorController* me, Sensor* TCSSwitchUp, Sensor* TCSSwitchDown, Sensor* TCSPot);
void MCM_calculateCommands(MotorController* mcm, TorqueEncoder* tps, BrakePressureSensor* bps);

void MCM_relayControl(MotorController* mcm, Sensor* HVILTermSense);
void MCM_inverterControl(MotorController* mcm, TorqueEncoder* tps, BrakePressureSensor* bps, ReadyToDriveSound* rtds);

void MCM_parseCanMessage(MotorController* mcm, IO_CAN_DATA_FRAME* mcmCanMessage);

ubyte1 MCM_getStartupStage(MotorController* me);
void MCM_setStartupStage(MotorController* me, ubyte1 stage);

#endif // _MOTORCONTROLLER_H
