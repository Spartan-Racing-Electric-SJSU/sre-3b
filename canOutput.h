#ifndef _CANOUTPUT_H
#define _CANOUTPUT_H

#include "can.h"

void canOutput_sendSensorMessages(void);
void canOutput_sendMCUControl(bool sendEvenIfNoChanges);


#endif //  _CANOUTPUT_H
