#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "APDB.h"
#include "IO_ADC.h"

#include "initializeVCU.h"

void inititalizeVCU(void)
{
    //Application Database, needed for TTC-Downloader
    appl_db =
    {
        0                      /* ubyte4 versionAPDB        */
        ,{ 0 }                    /* BL_T_DATE flashDate       */
                                  /* BL_T_DATE buildDate       */
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


    /* Initialize the IO driver (without safety functions) */
    //TODO: What does the VCU IO Driver do?
    IO_Driver_Init(NULL);

    //Power supplies ---------------------------------------------------
    //Analog sensor supplies
    IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_0, IO_POWER_ON);
    IO_POWER_Set(IO_ADC_SENSOR_SUPPLY_1, IO_POWER_ON);

    //Variable power supply (used by BPS)
    IO_POWER_Set(IO_SENSOR_SUPPLY_VAR, IO_POWER_14_5_V);    //IO_POWER_Set(IO_PIN_269, IO_POWER_8_5_V);

                                                            //Digital outputs ---------------------------------------------------
                                                            //IO_DO_Init(IO_DO_00);

                                                            //Digital PWM outputs ---------------------------------------------------
                                                            //We're not using these
    IO_PWM_Init(IO_PWM_07, 700, TRUE, FALSE, 0, FALSE, NULL);  //Temporary RTDS output

                                                               //ADC channels ---------------------------------------------------
                                                               //TPS
    IO_ADC_ChannelInit(IO_ADC_5V_00, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_01, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_1, NULL);

    //BPS
    IO_ADC_ChannelInit(IO_ADC_5V_02, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

    //Unused
    //IO_ADC_ChannelInit(IO_ADC_5V_03, IO_ADC_RATIOMETRIC, 0, 0, IO_ADC_SENSOR_SUPPLY_0, NULL);

    //Shockpots
    IO_ADC_ChannelInit(IO_ADC_5V_04, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_05, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_06, IO_ADC_RESISTIVE, 0, 0, 0, NULL);
    IO_ADC_ChannelInit(IO_ADC_5V_07, IO_ADC_RESISTIVE, 0, 0, 0, NULL);

    //PWD channels ---------------------------------------------------
    //Wheel Speed Sensors (Why is this called PWD?
    IO_PWD_FreqInit(IO_PWD_08, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?
    IO_PWD_FreqInit(IO_PWD_09, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?
    IO_PWD_FreqInit(IO_PWD_10, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?
    IO_PWD_FreqInit(IO_PWD_11, IO_PWD_RISING_VAR);  //Is there a reason to look for rising vs falling edge?

    //----------------------------------------------------------------------------
    // TODO: Initial Power-up functions
    //----------------------------------------------------------------------------
    //TODO: Read calibration data from EEPROM?
    //TODO: Run calibration functions?
    //TODO: Power-on error checking?

}
