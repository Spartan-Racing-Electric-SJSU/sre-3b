#ifndef _SAFETY_H
#define _SAFETY_H

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "IO_Driver.h"
#include "sensors.h"

typedef struct _SafetyChecker SafetyChecker;

SafetyChecker* SafetyChecker_new(void);
void SafetyChecker_update(SafetyChecker* me, TorqueEncoder* tps, BrakePressureSensor* bps);
bool SafetyChecker_allSafe(SafetyChecker* me);

#endif //  _TORQUEENCODER_H