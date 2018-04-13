/**************************************************************************
 *  XC-2000 IO-LIB
 *  Test Module
 **************************************************************************
 *  EEPROM driver
 *
 *  Writes data to EEPROM an reads it back again
 *
 **************************************************************************/

#include "IO_Driver.h"
#include "IO_EEPROM.h"
#include "IO_ADC.h"
#include "IO_DIO.h"
#include "IO_RTC.h"
#include "APDB.h"

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

void main (void)
{
    ubyte1 data=0;
    ubyte4 timestamp;
    ubyte1 flag=0;

    IO_Driver_Init(NULL); ///needed to initialize eeprom? see io_eeprom.h

    // Writing F for min resistance
    // state = 1;
    // Writing F/2 for mid resistance
    // state = 2;
    // Writing 0 for max resistance
    // state = 3;

    // TODO
    // 1 read the EEPROM values from hardware
    // 2 hardware transferred to cache
    // 3 cache manipulates that data "Hello"
    // 4 cache transfers to hardware EEPROM

    while(1)
    {
        IO_RTC_StartTime(&timestamp);
        IO_Driver_TaskBegin();

        if (IO_EEPROM_GetStatus() == IO_E_OK)
        {
            if(flag==0)
            {
                IO_EEPROM_Read(2,1,&data);
                flag=1;
            }
            else if(flag==1)
            {
                IO_EEPROM_Write(32,1,&data);
                flag=2;
            }
        }
        IO_Driver_TaskEnd();
    }
}

