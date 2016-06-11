#include <stdlib.h>
#include "IO_Driver.h"
//#include "IO_DIO.h"
//#include "IO_PWM.h"

#include "serial.h"
#include "sensors.h"
#include "cooling.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "bms.h"

//All temperatures in C
CoolingSystem* CoolingSystem_new(SerialManager* serialMan)
{
    CoolingSystem* CS_obj = (CoolingSystem*)malloc(sizeof(struct _CoolingSystem));
    SerialManager* sm = serialMan;

    //Cooling systems:
    //Water pump (motor, controller) - PWM
    float4 waterPumpMinPercent = 0.2;
    sbyte1 waterPumpLow = 25;  //Start ramping beyond min at this temp
    sbyte1 waterPumpHigh = 40;
    float4 waterPumpPercent = .2;

    //PP fans (motor, radiator) - Relay
    //Motor fan + radiator on same circuit
    sbyte1 motorFanLow = 30;    //Turn off BELOW tuhis point
    sbyte1 motorFanHigh = 32;    //Turn on at this temperature
    bool motorFanState = TRUE;        //float4 motorFanPercent;
    
    //Radiator fan
    //sbyte1 radiatorFanLo = 30; //Turn off BELOW tuhis point
    //sbyte1 radiatorFanHigh = 32; //Turn on at this temperature
    //bool motorFanState;

    //Battery fans (batteries) - Relay
    sbyte1 batteryFanLow = 32;  //Turn off BELOW tuhis point
    sbyte1 batteryFanHigh = 35;  //Turn on at this temperature
    bool batteryFanState = TRUE;      //float4 batteryFanPercent;
    
    return CS_obj;
}

//-------------------------------------------------------------------
// Cooling system calculations - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_calculations(CoolingSystem* me, sbyte1 motorControllerTemp, sbyte1 motorTemp, sbyte1 batteryTemp)
{


    //Water pump ------------------
    //Water pump PWM protocol unknown
    if (motorControllerTemp >= me->waterPumpHigh || motorTemp >= me->waterPumpHigh)
    {
        me->waterPumpPercent = 1;
    }
    else if (motorControllerTemp < me->waterPumpLow && motorTemp < me->waterPumpLow)
    {
        me->waterPumpPercent = .2;
    }
    else
    {
        me->waterPumpPercent = .2 + .8 * getPercent(max(motorControllerTemp, motorTemp), me->waterPumpLow, me->waterPumpHigh, TRUE);
    }

    //ubyte1* tempMsg[25];
    //sprintf(tempMsg, "MCM temp: %d\n", motorControllerTemp);
    //SerialManager_send(me->sm, tempMsg);
    //sprintf(tempMsg, "Motor temp: %d\n", motorTemp);
    //SerialManager_send(me->sm, tempMsg);
    //sprintf(tempMsg, "Batt temp: %d\n\n", batteryTemp);
    //SerialManager_send(me->sm, tempMsg);
    
    //Motor fan / rad fan
    if (me->motorFanState == FALSE)
    {
        if ((motorControllerTemp >= me->motorFanHigh) || (motorTemp >= me->motorFanHigh))
        {
            me->motorFanState = TRUE;
            SerialManager_send(me->sm, "Turning motor fans on.\n");
        }
    }
    else  //motor fan is on
    {
        if ((motorControllerTemp < me->motorFanLow) && (motorTemp < me->motorFanLow))
        {
            me->motorFanState = FALSE;
            SerialManager_send(me->sm, "Turning motor fans off.\n");
        }
    }

    //Battery fans
    if (me->batteryFanState == TRUE)
    {
        if (batteryTemp < 30)
        {
            me->batteryFanState = FALSE;
            SerialManager_send(me->sm, "Turning battery fans off.\n");
        }
    }
    else //fans are off
    {
        if (batteryTemp >= 32)
        {
            me->batteryFanState = TRUE;
            SerialManager_send(me->sm, "Turning battery fans on.\n");
        }
    }
    me->batteryFanState = FALSE;
    me->motorFanState = FALSE;
}


//    // Turn on FANS
//    //IO_DO_Init(IO_DO_06); 
//    IO_DO_Set(IO_DO_06, TRUE); //pin 142 - sending 12V 
//    IO_PWM_SetDuty(IO_PWM_05, 0, NULL);  //Water pump
//}
//else
//{
//    IO_DO_Set(IO_DO_06, FALSE);
//}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void CoolingSystem_enactCooling(CoolingSystem* me)
{
    //Send PWM control signal to water pump
    Light_set(Cooling_waterPump, me->waterPumpPercent);
    Light_set(Cooling_motorFans, me->motorFanState == TRUE ? 1 : 0);
    Light_set(Cooling_batteryFans, me->batteryFanState == TRUE ? 1 : 0);

}