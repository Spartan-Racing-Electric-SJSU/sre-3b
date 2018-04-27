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

#include "IO_Driver.h"
#include "IO_RTC.h"
#include "APDB.h"
#include "sensors.h"
#include "brakePressureSensor.h"

//From brakePressureSensor.h
#include "mathFunctions.h"
#include <stdlib.h>  
#include <math.h>

//From sensors.c
#include "IO_ADC.h"
#include "IO_PWD.h"
#include "IO_PWM.h"
#include "IO_DIO.h"

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

void main (void)
{
    ubyte4 timestamp;
    // TODO: Local variables, static variables, or dynamic variables declared here

    IO_Driver_Init( NULL );

    while(1)
    {
        IO_RTC_StartTime(&timestamp);

        IO_Driver_TaskBegin();

        // TODO: 

        IO_Driver_TaskEnd();
    }
}

// TODO: Function definitions->header file works too 