#ifndef _SAFETY_H
#define _SAFETY_H

#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "IO_Driver.h"
#include "sensors.h"

typedef enum { CHECK_tpsOutOfRange, CHECK_bpsOutOfRange
			   , CHECK_tpsOpenOrShort, CHECK_bpsOpenOrShort
			   , CHECK_tpsNotCalibrated, CHECK_bpsNotCalibrated 
			   , CHECK_tpsOutOfSync, CHECK_tpsbpsImplausible
			 } SafetyCheck;

typedef struct _SafetyChecker SafetyChecker;

SafetyChecker* SafetyChecker_new(void);
void SafetyChecker_update(SafetyChecker* me, TorqueEncoder* tps, BrakePressureSensor* bps);
bool SafetyChecker_allSafe(SafetyChecker* me);
bool SafetyChecker_getError(SafetyChecker* me, SafetyCheck check);

#endif //  _TORQUEENCODER_H