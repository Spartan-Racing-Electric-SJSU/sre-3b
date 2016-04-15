#ifndef _CANMANAGER_H
#define _CANMANAGER_H

#include "IO_Driver.h" 
#include "IO_CAN.h"

#include "avlTree.h"

typedef struct _CanManager CanManager;

typedef struct _CanMessageNode CanMessageNode;

CanManager* CanManager_new(ubyte4 defaultMinSendDelay, ubyte4 defaultMaxSendDelay);
void CanManager_setupMessage(ubyte2 messageID, ubyte4 timeBetweenMessages_Min, ubyte4 timeBetweenMessages_Max);


#endif // _CANMANAGER_H is defined