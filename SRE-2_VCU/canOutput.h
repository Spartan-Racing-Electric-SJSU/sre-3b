#ifndef _CANOUTPUT_H
#define _CANOUTPUT_H

#include "can.h"

//extern IO_CAN_DATA_FRAME *canMessages;

void canOutput_sendSensorMessages(void);
void canOutput_sendMCUControl(void);


#endif //  _CANOUTPUT_H