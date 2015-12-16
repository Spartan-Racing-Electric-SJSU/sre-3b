#include "IO_Driver.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"

//Import extrenal variables
//extern Sensor Sensor_WPS_FR; // = { 3 };

extern MotorController MCU0;
//Update the MCU object from its CAN messages
void motorController_setCommands() 
{
    //Temp hardcode
    MCU0.commands.enableDischarge = FALSE;

    //MCU0.commands.direction = 0;

    //Set Torque/Inverter control
    if (Sensor_WPS_FL.fresh == FALSE)
    {
        MCU0.commands.requestedTorque = 0;
    }
    else
    {
        MCU0.commands.requestedTorque = 30 * getPercent(Sensor_WPS_FL.sensorValue, 500, 2500, TRUE);
    }

    //If lockout has not been disabled
    //if (MCU0.lockoutDisabled == FALSE)
    if (MCU0.lockoutStatus == ENABLED) {
        MCU0.commands.enableInverter = FALSE;     //This tells the MCU to disable the lockout
    }
    else  //Lockout has been removed
    {
        //if (MCU0.inverterEnabled == FALSE)
        if (MCU0.inverterStatus == DISABLED) {
            MCU0.commands.enableInverter = TRUE;
        }
        else {
            MCU0.commands.enableInverter = TRUE;
        }
    }
    
    
}