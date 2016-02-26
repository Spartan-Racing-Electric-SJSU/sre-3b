#ifndef _CANOUTPUT_H
#define _CANOUTPUT_H

#include "can.h"
#include "motorController.h"

void canOutput_sendSensorMessages(void);
void canOutput_sendMCUControl(MotorController* mcm, bool sendEvenIfNoChanges);
//void canOutput_sendStatusMessages(MotorController* mcm);

#endif //  _CANOUTPUT_H
