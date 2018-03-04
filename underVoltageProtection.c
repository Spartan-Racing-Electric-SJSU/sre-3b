#include "underVoltageProtection.h"


/* Global variables */

/*In order for the ADC value to keep being read, this value must be true.
* Initially set to FALSE, but it initialization clears well, this variable will be set to TRUE
* If at any point anything goes wrong with the ADC pin or channel, this will be set to FALSE and the program will not read any value
*/
bool ADC_Channel_OK = FALSE; 

/*In order for the PWM value to keep being read, this value must be true.
* Initially set to FALSE, but it initialization clears well, this variable will be set to TRUE
* If at any point anything goes wrong with the PWM pin or channel, this will be set to FALSE and the program will not read any value
*/
bool PWM_Channel_OK = FALSE; 

IO_ErrorType ADC_Pin_Status = NULL; //Stores the current IO_ErrorType of the ADC pin
IO_ErrorType PWM_Pin_Status = NULL; //Stores the current IO_ErrorType of the PWM pin

LVBattery.warningBatteryConstant=0;
LVBattery.minBatteryThreshold= 12.8; 
LVBattery.maxBatteryThreshold= 14.8;

LVBattery.batteryTemperature= NULL;
LVBattery.warningTempConstant= NULL;
LVBattery.maxTempThreshold= NULL;
LVBattery.minTempThreshold= NULL;


 /** FUNCTION DEFINITIONS **/
bool ADC_Channel_OK( IO_ErrorType status ) {
	switch(status) 
	{
		case IO_E_OK:
			ADC_Channel_OK_Value = TRUE;
			break;
		case IO_E_INVALID_CHANNEL_ID:
			ADC_Channel_OK_Value = FALSE;
			break;
		case IO_E_NULL_POINTER:
			ADC_Channel_OK_Value = FALSE;
			break;
		case IO_E_FET_PROTECTION:
			ADC_Channel_OK_Value = FALSE;
			break;
		case IO_E_UNKNOWN:
			ADC_Channel_OK_Value = FALSE;
			break;
		case IO_E_CHANNEL_NOT_CONFIGURED:
			ADC_Channel_OK_Value = FALSE;
			break;
		default:
			ADC_Channel_OK_Value = FALSE;
			break;
	}
	return ADC_Channel_OK_Value;
}

bool PWM_Channel_OK( IO_ErrorType status ) {
	switch(status) 
	{
		case IO_E_OK:
			PWM_Channel_OK = TRUE;
			break;
		case IO_E_INVALID_CHANNEL_ID:
			PWM_Channel_OK = FALSE;
			break;
		case IO_E_NULL_POINTER:
			PWM_Channel_OK = FALSE;
			break;
		case IO_E_FET_PROTECTION:
			PWM_Channel_OK = FALSE;
			break;
		case IO_E_UNKNOWN:
			PWM_Channel_OK = FALSE;
			break;
		case IO_E_CHANNEL_NOT_CONFIGURED:
			PWM_Channel_OK = FALSE;
			break;
		default:
			PWM_Channel_OK = FALSE;
			break;
	}
	return PWM_Channel_OK;
}

float4 getLV_Voltage( void ){ //returns the voltage of the LV Battery 
	return LVBattery.currentBatteryLevel;
}

float4 getLV_Temperature( void ) { //returns the temperature of the LV Battery
	return LVBattery.batteryTemperature;
}

float4 motorcontroller_Voltage( void ){ //returns the voltage of the motor controller
	return (me->DC_Voltage);
}

float4 motorcontroller_Current( void ){ //returns the current of the motor controller
	return (me->DC_Current);
}

float4 motorcontroller_Temperature( void ){ //returns the temperature of the motor controller
	return MCM_getTemp(mcm0); 
}
void cooling_Regulate( CoolingSystem* me, sbyte2 motorControllerTemp ){ //regulates cooling based on undervoltage and temperature of the motors and the motor controller
	
}
