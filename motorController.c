#include "IO_Driver.h"
#include "motorController.h"
#include "outputCalculations.h"
#include "sensors.h"

//Import extrenal variables
//extern Sensor Sensor_WPS_FR; // = { 3 };

extern MotorController MCU0;
//Update the MCU object from its CAN messages
void motorController_setCommands() 
{
    //SET THE TORQUE to 10 Nm * reistance value HARD CODED TEMPORARILY
    if (Sensor_WPS_FL.fresh == FALSE)
    {
        MCU0.commands.requestedTorque = 0;
    }
    else
    {
        MCU0.commands.requestedTorque = 60 * getPercent(Sensor_WPS_FL.sensorValue, 500, 2500, TRUE);
        //MCU0.commands.requestedTorque = Sensor_WPS_FL.sensorValue;
    }
    
    //Temp hardcode
    MCU0.commands.enableDischarge = FALSE;
    
    //If lockout has not been disabled
    if (MCU0.lockoutDisabled == FALSE)
    {
        MCU0.commands.enableInverter = FALSE;
        //MCU0.commands.enableDischarge = FALSE;
        //MCU0.commands.direction = 0;
        //MCU0.commands.requestedTorque = 0;
    }
    else  //Lockout has been removed
    {
        if (MCU0.inverterEnabled == FALSE)
        {
            MCU0.commands.enableInverter = TRUE;
            //MCU0.commands.enableDischarge = FALSE;
            //MCU0.commands.direction = 0;
            //MCU0.commands.requestedTorque = 0;
        }
        else
        {
            MCU0.commands.enableInverter = TRUE;
            //MCU0.commands.enableDischarge = FALSE;
            
        }
    }
}


void testTorque(void) {
    //    MCU0.commands.requestedTorque = 100 * getPercent((float4)Sensor_WPS_FR.sensorValue, 500, 2500, TRUE);
}
