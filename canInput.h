#ifndef _CANINPUT_H
#define _CANINPUT_H

#include "can.h"
#include "motorController.h"
#include "bms.h"

void canInput_readMessages(MotorController* mcm, BatteryManagementSystem* bms);



#endif //  _CANINPUT_H
