#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
#include "IO_PWM"
#include "underVoltageProtection.h"


APDB appl_db =
{ 0 /* ubyte4 versionAPDB */
, {0} /* BL_T_DATE flashDate */
/* BL_T_DATE buildDate */
, { (ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
, 0 /* ubyte4 nodeType */
, 0 /* ubyte4 startAddress */
, 0 /* ubyte4 codeSize */
, 0 /* ubyte4 legacyAppCRC */
, 0 /* ubyte4 appCRC */
, 1 /* ubyte1 nodeNr */
, 0 /* ubyte4 CRCInit */
, 0 /* ubyte4 flags */
, 0 /* ubyte4 hook1 */
, 0 /* ubyte4 hook2 */
, 0 /* ubyte4 hook3 */
, APPL_START /* ubyte4 mainAddress */
, {0, 1} /* BL_T_CAN_ID canDownloadID */
, {0, 2} /* BL_T_CAN_ID canUploadID */
, 0 /* ubyte4 legacyHeaderCRC */
, 0 /* ubyte4 version */
, 500 /* ubyte2 canBaudrate */
, 0 /* ubyte1 canChannel */
, {0} /* ubyte1 reserved[8*4] */
, 0 /* ubyte4 headerCRC */
};
volatile ubyte4 count = 0;
ubyte4 timestamp = 0;



/* Function initialization */
bool ADC_Channel_OK( IO_ErrorType status );
bool PWM_Channel_OK( IO_ErrorType status );
float4 getLV_Voltage();
float4 getLV_Temperature(); 


float4 motorcontroller_Voltage();
float4 motorcontroller_Current();
float4 motorcontroller_Temperature();
float4 cooling_Voltage();
float4 cooling_Temperature();

/* Global variables */
bool ADC_Channel_OK = FALSE; //In order for the adc value keep being read, this value must be true
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

void main(void)
{
	//Initialize the driver
	IO_Driver_Init( NULL );
	IO_DI_DeInit( IO_ADC_5V_02 ); //TODO: change the IO_DI_DeInit to the right input and check with tim to makes sure all arguments are correct
	
	/* ADC CHANNEL INITIALIZATION */
	IO_ErrorType ADC_Pin_Status = IO_ADC_ChannelInit (IO_ADC_5v_04, //adc channel
	                    IO_ADC_ABSOLUTE, //type. Absolute means normal voltage input
	                    IO_ADC_RANGE_5V, //range. IO_ADC_RANGE_5V means 0-5V input
	                    IO_ADC_PU_10K, // pupd. IO_ADC_PU_10K means pull up resistor
	                    IO_ADC_SENSOR_SUPPLY_0, //sensor supply. Unclear on this one
	                    IO_ADC_SAFETY_CONF const * const safety_conf ); // Safety_conf. also unclear on this one
	
	/* POSSIBLE RETURN VALUES FOR ADC_Pin_Status 
 	* \return IO_ErrorType:
 	* \retval IO_E_OK                          everything fine
 	* \retval IO_E_INVALID_CHANNEL_ID          the channel id does not exist
 	* \retval IO_E_INVALID_PARAMETER           parameter is out of range
 	* \retval IO_E_CHANNEL_BUSY                the ADC input channel is currently used by another function
 	* \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing  */
	IO_ADC_ChannelDeInit( ubyte1 adc_channel );

	IO_ErrorType PWM_Pin_Status = IO_PWM_Init( IO_PWM_02, //IO_PWM_02 is the channel for the brake light
                        					   50, //???
                        					   TRUE, //???
                        					   bool cur_measurement, //???
                        					   ubyte1 cur_channel, //???
                        					   bool diag_margin, //???
                        					   IO_PWM_SAFETY_CONF const * const safety_conf ); //???
	/* POSSIBLE RETURN VALUES FOR PWM_Pin_Status
	* \return IO_ErrorType:
	* \retval IO_E_OK                          everything fine
	* \retval IO_E_CHANNEL_BUSY                the PWM output channel or the timer input channel is
	*                                          currently used by another function
	* \retval IO_E_INVALID_CHANNEL_ID          the PWM output channel or the timer input channel
	*                                          id don't exist
	* \retval IO_E_INVALID_PARAMETER           a given parameter is out of range
	* \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing
	*
	*/
    IO_PWM_DeInit( IO_PWM_02 );


	
	while (1)
	{
	
		IO_RTC_StartTime(&timestamp);
		

		//While the ADC channel doesn't return an error, the ADC will read the value
		if( ADC_Channel_OK(ADC_pin_Status) == TRUE ) 
		{
			LVBattery.currentBatteryLevel = IO_ADC_Get(IO_ADC_5v_04, IO_ADC_ABSOLUTE, TRUE); 
		}
		
		if(LVBattery.currentBatteryLevel > LVBattery.maxBatteryThreshold) {
			
			IO_PWM_SetDuty(IO_PWM_02,
                           dutyCycle, //Should be 100% duty cycle input
                           NULL); //Shouldn't get duty cycle feedback
		}
		else {
			//turn off
		}

		
		IO_Driver_TaskEnd();
		while (IO_RTC_GetTimeUS(timestamp) < 50000);
		{
		}
	
}






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

float4 getLV_Voltage(){
	return LVBattery.currentBatteryLevel;
}

float4 getLV_Temperature() {
	return LVBattery.batteryTemperature
}
