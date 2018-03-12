//Undervoltage Detection and Preventative Measures
//TODO: lots and lots of documentation to take care of

#ifndef _UNDERVOLTAGE_H
#define _UNDERVOLTAGE_H

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

typedef struct underVoltage {

		BatteryManagementSystem* bms;

		float4 currentBatteryLevel; //gets information from the LV battery about it's current battery life (look at how other sensors extract voltage data from the LV battery)
		float4 warningBatteryConstant; // = batteryPercentage - 0.5; get voltage value from the LV battery and subtract it by a placeholder constant value
		int minBatteryThreshold; //a constant for the minimum battery threshold
		int maxBatteryThreshold;

		float4 batteryTemperature; //maximum threshold for the battery temperature
		float4 warningTempConstant; //= batteryTemperature + 0.5
		int maxTempThreshold;
		int minTempThreshold;

		IO_ErrorType ADC_Pin_Status;  //Stores the current IO_ErrorType of the ADC pin
		IO_ErrorType PWM_Pin_Status;  //Stores the current IO_ErrorType of the PWM pin

		bool PWM_Channel_OK;
		bool ADC_Channel_OK; 

} underVoltage_ ;


/**
*Don't know if we need this honestly????
*/
//  struct {

// 		float current; //gets feedback from the LV components on what its current is
// 		float voltage; //gets feedback from the LV components on what its voltage is


// } LVComponents; //motorcontroller, cooling, wheelSpeedsSensor, brakePressureSensor, dcu, steeringAngleSensor;



 //Constructor for initializing "me" variable
 underVoltage_* UnderVoltage_new( void );

 //global "me" pointer
 underVoltage_* me;



/******** FUNCTIONS ********/
 float4 getLV_Voltage( void );
 float4 getLV_Temperature( void ); 
 float4 motorcontroller_Voltage( void );
 float4 motorcontroller_Current( void );
 float4 motorcontroller_Temperature( void );
 float4 cooling_Voltage( void );
 float4 cooling_Temperature( void );
 float brakePressure_Voltage( void );
 float wheelSpeeds_Voltage( void );																
 bool ADC_Channel_OK( IO_ErrorType status );
 bool PWM_Channel_OK( IO_ErrorType status );
// /*Mutators*/
 //void setConstants( void );
 //void setToDash( void );

#endif
