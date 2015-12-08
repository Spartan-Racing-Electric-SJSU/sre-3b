//----------------------------------------------------------------------------
// VCU Subsystem Initializations
//----------------------------------------------------------------------------
// This is where we turn on the VCU's internal power supplies and sensors.
//
// The supplies/sensors and their parameters are defined in IO_ADC.h.
// Each sensor/ADC channel/etc has different parameters, so make sure to
// check the .h files, the examples, and the documentation!
//----------------------------------------------------------------------------

#ifndef _INITIALIZEVCU_H
#define _INITIALIZEVCU_H

//Application Database, needed for TTC-Downloader
APDB appl_db;

//Functions
void vcu_inititalizeVCU(void);
void vcu_inititalizeADC(void);

#endif //  _INITIALIZEVCU_H
