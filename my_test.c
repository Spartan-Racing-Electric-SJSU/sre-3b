/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  Example driver
 *
 *  Write your test code within the main function
 * 	Questions can be asked on slack for the driver or contact the author:
 *	t i m o t h y . l e @ s p a r t a n r a c i n g e l e c t r i c . o r g
 *
 **************************************************************************/

#include <stdio.h>
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "APDB.h"
#include "IO_DIO.h"
#include "IO_ADC.h"

// TODO: Your libraries here

/* DO NOT EDIT */
APDB appl_db =
          { 0                      /* ubyte4 versionAPDB        */
          , {0}                    /* BL_T_DATE flashDate       */
                                   /* BL_T_DATE buildDate                   */
          , { (ubyte4) (((((ubyte4) RTS_TTC_FLASH_DATE_YEAR) & 0x0FFF) << 0) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MONTH) & 0x0F) << 12) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_DAY) & 0x1F) << 16) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_HOUR) & 0x1F) << 21) |
                        ((((ubyte4) RTS_TTC_FLASH_DATE_MINUTE) & 0x3F) << 26)) }
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
          , {0, 1}                 /* BL_T_CAN_ID canDownloadID */
          , {0, 2}                 /* BL_T_CAN_ID canUploadID   */
          , 0                      /* ubyte4 legacyHeaderCRC    */
          , 0                      /* ubyte4 version            */
          , 500                    /* ubyte2 canBaudrate        */
          , 0                      /* ubyte1 canChannel         */
          , {0}                    /* ubyte1 reserved[8*4]      */
          , 0                      /* ubyte4 headerCRC          */
          };

// TODO: Global variables and function declarations->header file works too
volatile ubyte4 count = 0;
ubyte4 timestamp = 0;

void main (void)
{
    // TODO: Local variables, static variables, or dynamic variables declared here
    ubyte2 adc_val_1;
    bool di_val_0, adc_fresh_1;

    IO_Driver_Init( NULL );

    /* APPS */
    // Brown GND1 Pin 145
    // Red Supply Voltage1 Pin 136
    // Orange Sensor Output 1 Pin 152
    // Green GND2 Pin 146
    // Blue Supply Voltage Pin 147
    // Yellow Sensor Output 2 Pin 140

    // Pin 117 for Brake Light
    IO_DO_Init( IO_DO_14 );
    IO_DO_DeInit( IO_DO_14 );

    // Analog Input Pin 140 APPS
    IO_ADC_ChannelInit( IO_ADC_5V_01
                    , IO_ADC_RATIOMETRIC
                    , 0 //IO_ADC_RANGE_15V
                    , 0 //IO_ADC_PU_10K
                    , IO_ADC_SENSOR_SUPPLY_0
                    , NULL );
    IO_ADC_ChannelDeInit( IO_ADC_5V_01 ); 

    /* Variable Range Selection Outputs */
    // IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    // IO_POWER_Set (IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);
    // IO_POWER_Set (IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);

    while(1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();        

        // Turn on the brake light
            IO_DO_Set( IO_DO_14
                     , TRUE );
        // TODO: 
        bool adc = IO_ADC_Get( IO_ADC_5V_01
                  , &adc_val_1
                  , &adc_fresh_1 ) == IO_E_OK ? TRUE : FALSE;

        // di_val_0 = adc_val_1 > 0 ? TRUE : FALSE; // lowside output

        if (adc)
        {
            // Turn on the brake light
            IO_DO_Set( IO_DO_14
                     , TRUE );
        }        

        IO_Driver_TaskEnd();

        while (IO_RTC_GetTimeUS(timestamp) < 50000);
        {
        }
    }
}

// TODO: Function definitions->header file works too 
