//This will be the implementation file of the under voltage code
#include <stdio.h>


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

	IO_ErrorType PWM_Pin_Status = IO_PWM_Init( IO_PWM_02, 50, TRUE, bool cur_measurement, ubyte1 cur_channel, bool diag_margin, NULL ); 
/* POSSIBLE RETURN VALUES FOR PWM_Pin_Status
	* \return IO_ErrorType:
	* \retval IO_E_OK                          everything fine
	* \retval IO_E_CHANNEL_BUSY                the PWM output channel or the timer input channel is
	*                                          currently used by another function
	* \retval IO_E_INVALID_CHANNEL_ID          the PWM output channel or the timer input channel
	*                                          id don't exist
	* \retval IO_E_INVALID_PARAMETER           a given parameter is out of range
	* \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing 
	*/
    IO_PWM_DeInit( IO_PWM_02 );

	
	while (1)
	{
	
		IO_RTC_StartTime(&timestamp);
		

		//While the ADC channel and PWM channel doesn't return an error, the ADC will read the value and PWM write is allowed
		if( ADC_Channel_OK(ADC_pin_Status) == TRUE && PWM_Channel_OK(PWM_Pin_Status) == TRUE ) 
		{
			LVBattery.currentBatteryLevel = IO_ADC_Get(IO_ADC_5v_04, IO_ADC_ABSOLUTE, TRUE); 

				if(LVBattery.currentBatteryLevel - 0.5 < LVBattery.minBatteryThreshold)
				{
					IO_PWM_SetDuty(IO_PWM_02, 100, NULL); //100% duty cycle
				}
				else { IO_PWM_SetDuty(IO_PWM_02, 0, NULL); } //0% duty cycle
		}
			

		IO_Driver_TaskEnd();
		while (IO_RTC_GetTimeUS(timestamp) < 50000);
		{
		}
	
}