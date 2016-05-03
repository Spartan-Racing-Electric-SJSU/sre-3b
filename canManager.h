#ifndef _CANMANAGER_H
#define _CANMANAGER_H

#include "IO_Driver.h" 
#include "IO_CAN.h"

#include "avlTree.h"

typedef enum { CAN0_HIPRI, CAN1_LOPRI } CanChannel;

typedef struct _CanManager CanManager;

typedef struct _CanMessageNode CanMessageNode;

//Note: Sum of messageLimits must be < 128 (hardware only does 128 total messages)
CanManager* CanManager_new(ubyte2 can0_busSpeed, ubyte1 can0_read_messageLimit, ubyte1 can0_write_messageLimit
                         , ubyte2 can1_busSpeed, ubyte1 can1_read_messageLimit, ubyte1 can1_write_messageLimit
                         , ubyte2 defaultSendDelayMs);
IO_ErrorType CanManager_send(CanChannel channel, IO_CAN_DATA_FRAME[]* canMessages, ubyte ? canMessageCount, bool force);


#endif // _CANMANAGER_H is defined