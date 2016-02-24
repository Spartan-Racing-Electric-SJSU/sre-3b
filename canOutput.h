#ifndef _CANOUTPUT_H
#define _CANOUTPUT_H

#include "can.h"
#include "motorController.h"

void canOutput_sendSensorMessages(void);
void canOutput_sendMCUControl(MotorController* mcm, bool sendEvenIfNoChanges);


#endif //  _CANOUTPUT_H
