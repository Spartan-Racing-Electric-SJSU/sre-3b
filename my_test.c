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

// VCU
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_ADC.h"
// #include "IO_PWD.h"
// #include "IO_PWM.h"
#include "IO_DIO.h"
#include "IO_UART.h"
#include "APDB.h"

// Standard Libraries from C
// #include <stdlib.h>  
// #include <stdio.h>
// #include <math.h>

// SRE Drivers
#include "sensors.h"
#include "brakePressureSensor.h"
#include "sensorCalculations.h"
#include "mathFunctions.h"

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
  IO_DO_Set( IO_DO_08, TRUE); // Pin 120 as DO
  // TODO: Local variables, static variables, or dynamic variables declared here
  bool bench;
  ubyte4 timestamp; // Generic timer for sensors
  ubyte4 timer_bps = 0; // Timer for brake
  ubyte4 timer_all = 0; // Timer for whole program
  ubyte1 calibrationErrors; // counter for errors

  // Digital Input initialization for bench
  IO_DI_Init(IO_DI_06, IO_DI_PD_10K);
  IO_RTC_StartTime(&timestamp);
  while (IO_RTC_GetTimeUS(timestamp) < 55555)
  {
    IO_Driver_TaskBegin();

    //IO_DI (digital inputs) supposed to take 2 cycles before they return valid data
    IO_DI_Get(IO_DI_06, &bench);

    IO_Driver_TaskEnd();
    //TODO: Find out if EACH pin needs 2 cycles or just the entire DIO unit
    while (IO_RTC_GetTimeUS(timestamp) < 10000);   // wait until 10ms have passed
  }
  IO_DI_DeInit(IO_DI_06);

  IO_Driver_Init( NULL );
  BrakePressureSensor* bps = BrakePressureSensor_new();

  while(1)
  {
    IO_DO_Set( IO_DO_08, TRUE); // Pin 120 as DO
    // Gets time stamp
    IO_RTC_StartTime(&timer_all);
    // Tells OS to initialize task and be ready to run
    IO_Driver_TaskBegin();
    // Performs ADC read
    sensors_updateSensors();

    // TODO: 
    // Runs calibration after receiving local sensor data and external data from CAN
    if(timer_bps == 0)
    {
      IO_RTC_StartTime(&timer_bps);
    }
    else if(IO_RTC_GetTimeUS(timer_bps) >= 3000000)
    {
      BrakePressureSensor_startCalibration(bps, 5);
    }

    // Verify calibration values when cycle is not finished
    BrakePressureSensor_update(bps, bench);
    BrakePressureSensor_calibrationCycle(bps, &calibrationErrors);
    // Terminates task, ends life cycle and returns process to scheduler
    IO_Driver_TaskEnd();

    // Loops until timeout
    while (IO_RTC_GetTimeUS(timer_all) < 33000) // 1000 = 1ms
    {
        IO_UART_Task();  //The task function shall be called every SW cycle.
    }
  }
}

// TODO: Function definitions->header file works too 