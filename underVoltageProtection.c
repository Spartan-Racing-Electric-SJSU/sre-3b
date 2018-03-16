#include "underVoltageProtection.h"


underVoltage_* BatteryFaults_new( void ) 
{
	//Memory allocation
	underVoltage_* me = (underVoltage_*) malloc(sizeof(underVoltage_));

	me->ADC_Pin_Status = NULL; //Stores the current IO_ErrorType of the ADC pin
    me->PWM_Pin_Status = NULL; //Stores the current IO_ErrorType of the PWM pin

	/*
	* In order for the ADC value to keep being read, this value must be true.
	* Initially set to FALSE, but it initialization clears well, this variable will be set to TRUE
	* If at any point anything goes wrong with the ADC pin or channel, this will be set to FALSE and the program will not read any value
	*/
	me->ADC_Channel_OK = FALSE; 

	/*
	* In order for the PWM value to keep being read, this value must be true.
	* Initially set to FALSE, but it initialization clears well, this variable will be set to TRUE
	* If at any point anything goes wrong with the PWM pin or channel, this will be set to FALSE and the program will not read any value
	*/
	me->PWM_Channel_OK = FALSE;


	//initializing faults
	me->warningBatteryConstant= 0;
	me->minBatteryThreshold= 12.8; 
	me->maxBatteryThreshold= 14.8;

	me->batteryTemperature= NULL;
	me->warningTempConstant= NULL;
	me->maxTempThreshold= NULL;
	me->minTempThreshold= NULL;


	free(me);

	return me;
}



 /** FUNCTION DEFINITIONS **/
bool ADC_Channel_OK( IO_ErrorType status ) {
	switch(status) 
	{
		case IO_E_OK:
			me->ADC_Channel_OK = TRUE;
			break;
		case IO_E_INVALID_CHANNEL_ID:
			me->ADC_Channel_OK = FALSE;
			break;
		case IO_E_NULL_POINTER:
			me->ADC_Channel_OK = FALSE;
			break;
		case IO_E_FET_PROTECTION:
			me->ADC_Channel_OK = FALSE;
			break;
		case IO_E_UNKNOWN:
			me->ADC_Channel_OK = FALSE;
			break;
		case IO_E_CHANNEL_NOT_CONFIGURED:
			me->ADC_Channel_OK = FALSE;
			break;
		default:
			me->ADC_Channel_OK = FALSE;
			break;
	}
	return me->ADC_Channel_OK;
}

bool PWM_Channel_OK( IO_ErrorType status ) {
	switch(status) 
	{
		case IO_E_OK:
			me->PWM_Channel_OK = TRUE;
			break;
		case IO_E_INVALID_CHANNEL_ID:
			me->PWM_Channel_OK = FALSE;
			break;
		case IO_E_NULL_POINTER:
			me->PWM_Channel_OK = FALSE;
			break;
		case IO_E_FET_PROTECTION:
			me->PWM_Channel_OK = FALSE;
			break;
		case IO_E_UNKNOWN:
			me->PWM_Channel_OK = FALSE;
			break;
		case IO_E_CHANNEL_NOT_CONFIGURED:
			me->PWM_Channel_OK = FALSE;
			break;
		default:
			me->PWM_Channel_OK = FALSE;
			break;
	}
	return me->PWM_Channel_OK;

}

void UV_parseCanMessage(underVoltage_ *uv, IO_CAN_DATA_FRAME* uvCanMessage) { //cases with different IDs that represent UnderVoltage CAN messages 
	
	switch(uvCanMessage->id){
		case 0x700: //id that represents the warningTempConstant decreases past the minBatteryThreshold
			uv->warningTempConstant = ((ubyte2)uvCanMessage->data[5] << 8 | uvCanMessage->data[4]) / 10; //placeholder message
			break;

	}

}

float4 getLV_Voltage( void ){ //returns the voltage of the LV Battery 
	//return me->currentBatteryLevel;
	return 0;
}

float4 getLV_Temperature( void ) { //returns the temperature of the LV Battery
	//return me->batteryTemperature;
	return 0;
}

float4 motorcontroller_Voltage( void ){ //returns the voltage of the motor controller
	//return (me->DC_Voltage);
	return 0;
}

float4 motorcontroller_Current( void ){ //returns the current of the motor controller
	//return (me->DC_Current);
	return 0;
}

float4 motorcontroller_Temperature( void ){ //returns the temperature of the motor controller
	//return MCM_getTemp(mcm0); 
	return 0; //Stub
}

//void cooling_Regulate( CoolingSystem* me, sbyte2 motorControllerTemp ){ //regulates cooling based on undervoltage and temperature of the motors and the motor controller
	

//}