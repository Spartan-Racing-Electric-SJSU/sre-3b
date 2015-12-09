#include "IO_Driver.h"
#include "motorController.h"
//#include "outputCalculations.h"
//#include "sensors.h"

//Import extrenal variables
//extern Sensor Sensor_WPS_FR; // = { 3 };

extern MotorController MCU0;

//Update the MCU object from its CAN messages
void motorController_setCommands() 
{
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
            
            //SET THE TORQUE to 10 Nm * reistance value HARD CODED TEMPORARILY
//            MCU0.commands.requestedTorque = 100 * getPercent((float4)Sensor_WPS_FR.sensorValue, 500, 2500, TRUE);
        }
    }
}


void testTorque(void) {
    //    MCU0.commands.requestedTorque = 100 * getPercent((float4)Sensor_WPS_FR.sensorValue, 500, 2500, TRUE);
}
