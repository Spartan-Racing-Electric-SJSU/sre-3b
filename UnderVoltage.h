// UnderVoltage Protection handler
#ifdef _UNDERVOLTAGE_
#define _UNDERVOLTAGE_ 

#include <stdio.h>


char* errorMessage; //This will be used for error/fault messages


struct batteryFaults {

	//For battery levels
	const int minimumThresshold; //This will be 12.8
	float currentBatteryLevel;
	float warningBatteryConstant; //GetBatteryValue() - (0.5)

	//For temperature of the battery
	const int maxTempThresshold;
	float batteryTemperature;
	float warningTempConstant;

} LVBattery;

struct LVComponentFaults {

	float current;




} motorcontroller ;


struct _MotorController LVmotorController;



