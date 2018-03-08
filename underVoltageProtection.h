//Undervoltage Detection and Preventative Measures
//TODO: lots and lots of documentation to take care of

//#ifndef _UNDERVOLTAGE_H
//#define _UNDERVOLTAGE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_CAN.h"
#include "IO_PWM.h"
#include "IO_UART.h" //Potentially used in case CAN does not work
#include "bms.h"

struct _BatteryFaults{

BattteryManagementSystem* bms;

float4 currentBatteryLevel; //gets information from the LV battery about it's current battery life (look at how other sensors extract voltage data from the LV battery)
float4 warningBatteryConstant; // = batteryPercentage - 0.5; get voltage value from the LV battery and subtract it by a placeholder constant value
int minBatteryThreshold; //a constant for the minimum battery threshold
int maxBatteryThreshold;

float4 batteryTemperature; //maximum threshold for the battery temperature
float4 warningTempConstant; //= batteryTemperature + 0.5
int maxTempThreshold;
int minTempThreshold;

}; //LVBattery

struct LVComponent{

	float current; //gets feedback from the LV components on what its current is
	float voltage; //gets feedback from the LV components on what its voltage is


}; //motorcontroller, cooling, wheelSpeedsSensor, brakePressureSensor, dcu, steeringAngleSensor;


//struct _MotorController lvMotorController;

//motorcontroller.current

//functions for the undervoltage 


//Functions
/*Acessors*/

float4 getLV_Voltage();
float4 getLV_Temperature(); 

float4 motorcontroller_Voltage();
float4 motorcontroller_Current();
float4 motorcontroller_Temperature();

float4 cooling_Voltage();
float4 cooling_Temperature();


float brakePressure_Voltage();
float wheelSpeeds_Voltage();
																		

/*Mutators*/
void setConstants();
void setToDash();

//#endif
