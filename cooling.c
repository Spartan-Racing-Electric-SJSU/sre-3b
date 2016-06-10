
//#include "IO_DIO.h"
//#include "IO_PWM.h"

#include "sensors.h"
#include "cooling.h"
#include "motorController.h"
#include "bms.h"

//struct _BatteryManagementSystem {
//    
//
//};
//
//BatteryManagementSystem* BMS_new(ubyte2 canMessageBaseID){
//
//	BatteryManagementSystem* BMS_obj = (BatteryManagementSystem*)malloc(sizeof(struct _BatteryManagementSystem));
//	BMS_obj->canMessageBaseId = canMessageBaseID;
//	return BMS_obj;
//
//}

//-------------------------------------------------------------------
// Cooling system control - turns fans on/off, sends water pump PWM control signal
//Rinehart water temperature operating range: -30C to +80C before derating
//-------------------------------------------------------------------
void coolingSystemControl(BatteryManagementSystem* bms)
{
    //Cooling systems:
    //Water pump (motor, controller) - PWM
    sbyte1 waterPumpStartTemp = 25;
    sbyte1 waterPumpMaxTemp = 40;
    float4 waterPumpPercent;

    //PP fans (motor, radiator) - Relay
    //Motor fan + radiator on same circuit
    sbyte1 motorFanStart = 18; //Turn off BELOW tuhis point
    sbyte1 motorFanMax = 21;     // Turn on at this temperature
    bool motorFanState;
    //float4 motorFanPercent;

    //Battery fans (batteries) - Relay
    sbyte1 batteryFanStart = 25;
    sbyte1 batteryFanMax = 28;
    bool batteryFanState;
    //float4 batteryFanPercent;

    //Water pump ------------------
    //IO_PWM_SetDuty(IO_PWM_05, 0x3A, NULL);  //Water pump

    //if ((BMS_getPackTemp(bms) > 35))
    //{
    //    // Turn on FANS
    //    //IO_DO_Init(IO_DO_06); 
    //    IO_DO_Set(IO_DO_06, TRUE); //pin 142 - sending 12V 
    //    IO_PWM_SetDuty(IO_PWM_05, 0, NULL);  //Water pump
    //}
    //else
    //{
    //    IO_DO_Set(IO_DO_06, FALSE);
    //}
}

