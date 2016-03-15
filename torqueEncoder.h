#ifndef _TORQUEENCODER_H
#define _TORQUEENCODER_H

#include "IO_Driver.h"
#include "sensors.h"

typedef struct _TorqueEncoder
{
    bool bench;

    Sensor* tps0;
    Sensor* tps1;

    ubyte2 tps0_rawCalibMin;
    ubyte2 tps0_rawCalibMax;
    ubyte2 tps0_calibMin;
    ubyte2 tps0_calibMax;
    //float4 tps0_percent;

    ubyte2 tps1_rawCalibMin;
    ubyte2 tps1_rawCalibMax;
    ubyte2 tps1_calibMin;
    ubyte2 tps1_calibMax;
    //float4 tps1_percent;

    bool runCalibration;
    ubyte4 timestamp_calibrationStart;
    ubyte1 calibrationRunTime;

    bool calibrated;
    float4 percent;
} TorqueEncoder;

TorqueEncoder* TorqueEncoder_new(bool benchMode);
void TorqueEncoder_getIndividualSensorPercent(TorqueEncoder* me, ubyte1 sensorNumber, float4* percent);
void TorqueEncoder_resetCalibration(TorqueEncoder* me);
void TorqueEncoder_saveCalibrationToEEPROM(TorqueEncoder* me);
void TorqueEncoder_loadCalibrationFromEEPROM(TorqueEncoder* me);
void TorqueEncoder_startCalibration(TorqueEncoder* me, ubyte1 secondsToRun);
void TorqueEncoder_calibrationCycle(TorqueEncoder* me, ubyte1* errorCount);
void TorqueEncoder_plausibilityCheck(TorqueEncoder* me, ubyte1* errorCount, bool* fail);
void TorqueEncoder_getPedalTravel(TorqueEncoder* me, ubyte1* errorCount, float4* pedalPercent);

#endif //  _TORQUEENCODER_H