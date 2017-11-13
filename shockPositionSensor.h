/****************************************************************************
*  @file    shockPositionSensor.h
*  @brief   Shock Position Seonsor is same as a Linear Potentiometer
*           Position Sensor. It is used to monitor and track the linear
*           motion or position of a target
*
*           I choose the LPPS-050 model for the head file blue print
*           to fit the FSAE Suspension Rule (usable wheel travel of at least
*           50.8 mm (2 inches), see the engineering note book)
*           because the measuring range (stroke length) for this model is 2 inches.
*
*           The Max Input Voltage (VDC) is 24V?
*           Our control system voltage is 12V, so the input voltage is 12V?
*           Output: 0 to 100% of Input Voltage (potentiometer circuit)
*           The measuring range (stroke length)is 2 inches [50mm].
*           The Mechanical Stroke Length is + 3mm.
*
*  @author  Xiao Pan
*  @date    11/9/2017
*  @reference http://thesensorconnection.com/linear-travel-sensors/
*  all-linear-travel-sensors/shock-absorber-travel-sensor-rod-end-joints
*
*****************************************************************************/
#ifndef SHOCK_POSITION_SENSOR_H_
#define SHOCK_POSITION_SENSOR_H_

#include "IO_Driver.h"

#define MAX_V 12        /**< maximum output voltage 12VDC  */
#define MIN_V 0         /**< minimum output voltage 0VDC  */
#define MAX_mm 50       /**< maximum measuring range 50 mm*/
#define MIN_mm 0        /**< minimum measuring range 0 mm*/

ubyte4 shock_vtomm(ubyte1 output_v);
/**< convert output voltage to length compressed*/

#endif // MYHEADER_H_
