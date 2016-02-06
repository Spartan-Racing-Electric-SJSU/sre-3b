/*****************************************************************************
* Sensors
******************************************************************************
* bla bla bla.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-12-01 - Rusty Pedrosa - Changed loading of sensor data to switch
*                              statement inside of a loop
*****************************************************************************/

#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_ADC.h"
#include "IO_PWD.h"
#include "IO_PWM.h"
//#include "IO_DIO.h"

#include "sensors.h"
#include "mathFunctions.h"

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
//extern Sensor Sensor_BPS1;
extern Sensor Sensor_WSS_FL;
extern Sensor Sensor_WSS_FR;
extern Sensor Sensor_WSS_RL;
extern Sensor Sensor_WSS_RR;
extern Sensor Sensor_WPS_FL;
extern Sensor Sensor_WPS_FR;
extern Sensor Sensor_WPS_RL;
extern Sensor Sensor_WPS_RR;
extern Sensor Sensor_SAS;
extern Sensor Sensor_LVBattery;

extern Sensor Sensor_RTD_Button;
//TEMP BENCH SWITCHES
extern Sensor Sensor_TEMP_BrakingSwitch;


/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
//----------------------------------------------------------------------------
// Read sensors values from ADC channels
// The sensor values should be stored in sensor objects.
//----------------------------------------------------------------------------
void sensors_updateSensors(void)
{

    //TODO: Handle errors (using the return values for these Get functions)

    //TODO: RTDS

    //Torque Encoders ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_00, &Sensor_TPS0.sensorValue, &Sensor_TPS0.fresh);
    IO_ADC_Get(IO_ADC_5V_01, &Sensor_TPS1.sensorValue, &Sensor_TPS1.fresh);

    //Brake Position Sensor ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_02, &Sensor_BPS0.sensorValue, &Sensor_BPS0.fresh);

    //?? - For future use ---------------------------------------------------
    //IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

    //Shock pots ---------------------------------------------------
    IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FL.sensorValue, &Sensor_WPS_FL.fresh);
    IO_ADC_Get(IO_ADC_5V_05, &Sensor_WPS_FR.sensorValue, &Sensor_WPS_FR.fresh);
    IO_ADC_Get(IO_ADC_5V_06, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RL.fresh);
    IO_ADC_Get(IO_ADC_5V_07, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RR.fresh);


    //Wheel speed sensors ---------------------------------------------------
    IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FL.sensorValue);
    IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_FR.sensorValue);
    IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_RL.sensorValue);
    IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RR.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);


    //----------------------------------------------------------------------------
    //RTDS test - Temporary code only
    //----------------------------------------------------------------------------
    //Control the RTDS with a pot
    //Pot goes from 2.2 ohm to 4930 ohm
    //Note: There's a problem with the old RTDS where it plays sound
    //      even at 0 duty cycle / DO=FALSE.  Gotta figure out why this
    //      happens and if there's a problem with the VCU.

    /* Temporary comment out 
    //Hook up RTDS to pin 103
    float4 dutyPercent;  //Percent (some fraction between 0 and 1)
    ubyte2 dutyHex;      //percent * max value (FFFF)
    
    dutyPercent = getPercent((float4)Sensor_WPS_FL.sensorValue, 50, 4550, TRUE);

    //Set the volume level (0 to 65535.. or 0 to FFFF as seen by VCU)
    //dutyHex = 65535 * dutyPercent;       //becomes an integer
    dutyHex = 6553 * dutyPercent;       //becomes an integer
    dutyHex = (Sensor_WPS_FL.fresh == FALSE) ? 0 : dutyHex;  //Set to 0 if sensor reading is not fresh

    IO_PWM_SetDuty(IO_PWM_07, dutyHex, NULL);  //Pin 103
    */

}
