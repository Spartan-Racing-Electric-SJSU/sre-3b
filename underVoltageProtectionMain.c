//This will be the implementation file of the under voltage code
#include "testFile.h"

//Application Database, needed for TTC-Downloader
APDB appl_db =
    { 0                      /* ubyte4 versionAPDB        */
    ,{ 0 }                    /* BL_T_DATE flashDate       */
                          /* BL_T_DATE buildDate                   */
    ,{ (ubyte4)(((((ubyte4)RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
        ((((ubyte4)RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
    , 0                      /* ubyte4 nodeType           */
    , 0                      /* ubyte4 startAddress       */
    , 0                      /* ubyte4 codeSize           */
    , 0                      /* ubyte4 legacyAppCRC       */
    , 0                      /* ubyte4 appCRC             */
    , 1                      /* ubyte1 nodeNr             */
    , 0                      /* ubyte4 CRCInit            */
    , 0                      /* ubyte4 flags              */
    , 0                      /* ubyte4 hook1              */
    , 0                      /* ubyte4 hook2              */
    , 0                      /* ubyte4 hook3              */
    , APPL_START             /* ubyte4 mainAddress        */
    ,{ 0, 1 }                 /* BL_T_CAN_ID canDownloadID */
    ,{ 0, 2 }                 /* BL_T_CAN_ID canUploadID   */
    , 0                      /* ubyte4 legacyHeaderCRC    */
    , 0                      /* ubyte4 version            */
    , 500                    /* ubyte2 canBaudrate        */
    , 0                      /* ubyte1 canChannel         */
    ,{ 0 }                    /* ubyte1 reserved[8*4]      */
    , 0                      /* ubyte4 headerCRC          */
};

#define DUTY_LITTLE_STEP    0x100
#define DUTY_BIG_STEP       0x2000

volatile ubyte4 count = 0;
volatile ubyte2 duty = 0;
volatile ubyte2 dual_duty = 0x7FFF;
volatile ubyte2 duty_step = 0;
volatile sbyte2 dir = 0;

ubyte4 timestamp_startTime = 0;

//pot
ubyte2 adc_val_20;
bool adc_fresh_20;

void main(void)
{
	bool direction = TRUE;
	
	/***** DRIVER INITIALIZATION *****/
	IO_Driver_Init( NULL );

	/***** ADC CHANNEL INITIALIZATION *****/
	// IO_ErrorType ADC_Pin_Status =
	IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_ABSOLUTE, IO_ADC_RANGE_5V, IO_ADC_PU_10K, IO_ADC_SENSOR_SUPPLY_0, IO_E_OK); 
	IO_ADC_ChannelDeInit( IO_ADC_5V_04 );

	/***** PWM CHANNEL INITIALIZATION *****/
	//IO_ErrorType PWM_Pin_Status = 
	IO_PWM_Init( IO_PWM_02, 150, TRUE, FALSE, 0, TRUE, NULL ); 
	//IO_PWM_DeInit( IO_PWM_02 );

	/***** CAN CHANNEL INITIALIZATION *****/
	//IO_ErrorType IO_CAN_Init( ubyte1 channel, ubyte2 baudrate, ubyte1 tseg1, ubyte1 tseg2, ubyte1 sjw);
	//IO_CAN_DeInit( ubyte1 channel );

	// underVoltage_* myUV= UnderVoltage_new(); 
	// bool messageSent= FALSE;
	// bool fixed= TRUE;
	while (1)
	{
		IO_RTC_StartTime(&timestamp_startTime);

		IO_Driver_TaskBegin();

		//Potentiometer
		IO_ADC_Get(IO_ADC_5V_04, IO_ADC_ABSOLUTE, TRUE); 



		//LED LIGHT
		IO_PWM_SetDuty(IO_PWM_02, duty, NULL); 
		if (duty >= 0xFFFF)
        {
            dir = -DUTY_LITTLE_STEP;
        }
        if (duty <= 0x0)
        {
            dir = DUTY_LITTLE_STEP;
        }
        duty += dir;

        if ((count % 150) == 0)
        {
            direction = (direction == TRUE) ? FALSE : TRUE;
        }
        dual_duty += DUTY_LITTLE_STEP;

        if ((count % 150) == 0)
        {
            duty_step += DUTY_BIG_STEP;
        }
        count++;
	    


		// ADC_Channel_OK(ADC_Pin_Status, myUV);
		// PWM_Channel_OK(PWM_Pin_Status, myUV);

		// //While the ADC channel and PWM channel doesn't return an error, the ADC will read the value and PWM write is allowed
		// if( myUV->PWM_Channel_OK == TRUE && myUV->ADC_Channel_OK == TRUE ) 
		// {
		// 	myUV->currentBatteryLevel = IO_ADC_Get(IO_ADC_5V_04, IO_ADC_ABSOLUTE, TRUE); 

		// 		//If the the current battery level is just about to pass the threshold, turn on the brake light to signal undervoltage will happen
		// 		if((myUV->currentBatteryLevel - 0.4 < myUV->minBatteryThreshold) && !messageSent)
		// 		{
		// 			//turn on brake light, stores error type
		// 			PWM_Pin_Status= IO_PWM_SetDuty(IO_PWM_02, duty, NULL); 

		// 			//Wilson's code of sending a message to indicate undervolate goes here
		// 			UV_parseCanMessage(myUV, 0x700); //calls the UV CAN function to display CAN message
		// 			messageSent=TRUE;
		// 			fixed=FALSE;
		// 		} 
		// 		if(fixed)  
		// 		{
		// 			//brake light off and no message is necessary. Also stores error type
		// 			PWM_Pin_Status= IO_PWM_SetDuty(IO_PWM_02, 0, NULL); //0% duty cycle
		// 		} 
		// }
		IO_Driver_TaskEnd();

		while (IO_RTC_GetTimeUS(timestamp_startTime) < 50000);
		{
		}
	
    }
}

//TODO: Need to know if all of the driver/rtc stuff is set up correctly. e.g lines IO_Driver_TaskEnd() to the end of the "while" loop.