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
	

	/***** ADC CHANNEL INITIALIZATION *****/
	IO_ErrorType ADC_Pin_Status = IO_ADC_ChannelInit (IO_ADC_5v_04, IO_ADC_ABSOLUTE, IO_ADC_RANGE_5V, IO_ADC_PU_10K, IO_ADC_SENSOR_SUPPLY_0, NULL); 
	IO_ADC_ChannelDeInit( IO_ADC_5v_04 );


	/***** PWM CHANNEL INITIALIZATION *****/
	IO_ErrorType PWM_Pin_Status = IO_PWM_Init( IO_PWM_02, 50, TRUE, bool cur_measurement, ubyte1 cur_channel, bool diag_margin, NULL ); 
	IO_PWM_DeInit( IO_PWM_02 );


	/***** CAN CHANNEL INITIALIZATION *****/
	IO_ErrorType IO_CAN_Init( ubyte1 channel, ubyte2 baudrate, ubyte1 tseg1, ubyte1 tseg2, ubyte1 sjw);
	IO_CAN_DeInit( ubyte1 channel );

	
	while (1)
	{
		IO_RTC_StartTime(&timestamp);
		

		//While the ADC channel and PWM channel doesn't return an error, the ADC will read the value and PWM write is allowed
		if( ADC_Channel_OK(ADC_pin_Status) == TRUE && PWM_Channel_OK(PWM_Pin_Status) == TRUE ) 
		{
			me->currentBatteryLevel = IO_ADC_Get(IO_ADC_5v_04, IO_ADC_ABSOLUTE, TRUE); 

				if(me->currentBatteryLevel - 0.5 < me->minBatteryThreshold)
				{
					IO_PWM_SetDuty(IO_PWM_02, 100, NULL); //100% duty cycle


					//Wilson's code of sending a message to indicate undervolate goes here!


				}  
				else 
				{
					IO_PWM_SetDuty(IO_PWM_02, 0, NULL); //0% duty cycle
				} 
		}
			

		IO_Driver_TaskEnd();
		while (IO_RTC_GetTimeUS(timestamp) < 50000);
		{
		}
	
}