//Undervoltage Detection and Preventative Measures
#ifndef _UNDERVOLTAGE_H
#define _UNDERVOLTAGE_H

#include <stdio.h>

char* errorMessage; //the char pointer is a global variable that is used for any error message related to LV systems

typedef struct batteryFaults{

	float currentBatteryLevel; //gets information from the LV battery about it's current battery life (look at how other sensors extract voltage data from the LV battery)
	float warningBatteryConstant // = batteryPercentage - 0.5; get voltage value from the LV battery and subtract it by a placeholder constant value
	int minBatteryThreshold; //a constant for the minimum battery threshold

	float batteryTemperature; //maximum threshold for the battery temperature
	float warningTempConstant; //= batteryTemperature + 0.5;
	int maxTempThreshold

} lvBattery;

typedef struct LVComponentFaults{

	float current; //gets feedback from the LV components on what its current is
	float voltage; //gets feedback from the LV components on what its voltage is


} motorcontroller, cooling, wheelSpeedsSensor, brakePressureSensor, dcu, steeringAngleSensor;


struct _MotorController lvMotorController;

//motorcontroller.current

//functions for the undervoltage 


//Functions
/*Acessors*/

float getLV_Voltage()
float getLV_Temperature() 

float motorcontroller_Voltage()
float motorcontroller_Current()
float motorcontroller_Temperature()

float cooling_Voltage()
float cooling_Temperature()



float dcu_Voltage()

float brakePressure_Voltage()
float wheelSpeeds_Voltage();
																					/*	LVBattery.warningBatteryConstant; 
																						LVBattery.minBatteryThreshold = 12.8;

																						LVBattery.warningTempConstant = 90;
																						LVBattery.maxTempThreshold;

																						motorcontroller. */


																						//return lvMotorController.DC_Voltage;


/*Mutators*/
void setConstants()
void setToDash()
