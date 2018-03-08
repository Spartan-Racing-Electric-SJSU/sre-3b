//This will be the implementation file of the under voltage code
#include "underVoltageProtection.c"

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

	 /* CAN CHANNEL INITIALIZATION */
	/*	IO_ErrorType IO_CAN_Init( ubyte1 channel
                         , ubyte2 baudrate
                         , ubyte1 tseg1
                         , ubyte1 tseg2
                         , ubyte1 sjw );

	 /* SERIAL INITIALIZATION */
	  IO_ErrorType IO_UART_Init( ubyte1 channel
                          , ubyte4 baudrate
                          , ubyte1 dbits
                          , ubyte1 par
                          , ubyte1 sbits);
	
	/* POSSIBLE RETURN VALUES FOR ADC_Pin_Status 
 	* \return IO_ErrorType:
 	* \retval IO_E_OK                          everything fine
 	* \retval IO_E_INVALID_CHANNEL_ID          the channel id does not exist
 	* \retval IO_E_INVALID_PARAMETER           parameter is out of range
 	* \retval IO_E_CHANNEL_BUSY                the ADC input channel is currently used by another function
 	* \retval IO_E_DRV_SAFETY_CONF_NOT_CONFIG  Global safety configuration is missing  */
	IO_ADC_ChannelDeInit( ubyte1 adc_channel );

	/* CAN CHANNEL DENINITIALIZATION */
	IO_CAN_DeInit( ubyte1 );

	/* SERIAL DEINITIALIZATION */
	IO_UART_DeInit(ubyte1 channel);


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


    //Initialization of UART goes here

    //Deinitialization of UART goes here

	
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


					//Wilson's code of sending a message to indicate undervolate goes here!


				}
				else { IO_PWM_SetDuty(IO_PWM_02, 0, NULL); } //0% duty cycle
		}
			

		IO_Driver_TaskEnd();
		while (IO_RTC_GetTimeUS(timestamp) < 50000);
		{
		}
	
}