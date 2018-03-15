#ifndef _CANMANAGER_H
#define _CANMANAGER_H

#include "IO_Driver.h" 
#include "IO_CAN.h"

/* include underVoltage or any other files here */
#include "underVoltageProtection.h"

typedef enum { CAN0_HIPRI, CAN1_LOPRI } CanChannel;
//CAN0: 48 messages per handle (48 read, 48 write)
//CAN1: 16 messages per handle

typedef struct _CanManager CanManager;

typedef struct _CanMessageNode CanMessageNode;

//Note: Sum of messageLimits must be < 128 (hardware only does 128 total messages)
CanManager* CanManager_new(ubyte2 can0_busSpeed, ubyte1 can0_read_messageLimit, ubyte1 can0_write_messageLimit
                         , ubyte2 can1_busSpeed, ubyte1 can1_read_messageLimit, ubyte1 can1_write_messageLimit
                         , ubyte4 defaultSendDelayus, SerialManager* sm);
IO_ErrorType CanManager_send(CanManager* me, CanChannel channel, IO_CAN_DATA_FRAME canMessages[], ubyte1 canMessageCount);

//Reads and distributes can messages to their appropriate subsystem objects so they can updates themselves
void CanManager_read(CanManager* me, CanChannel channel, underVoltage_ *uv);

/* ADC voltage output that helps with indicating the voltage level */
void canOutput_sendSensorMessages(CanManager* me);

/* Motor Controller Unit CAN Control */
//void canOutput_sendMCUControl(CanManager* me, MotorController* mcm, bool sendEvenIfNoChanges);

/* Debug messaging for Under Voltage */
void canOutput_sendDebugMessage(CanManager* me, underVoltage_ *uv);

/* Interprets the status of the CAN channel at the time of invocation */
ubyte1 CanManager_getReadStatus(CanManager* me, CanChannel channel);

#endif // _CANMANAGER_H is defined