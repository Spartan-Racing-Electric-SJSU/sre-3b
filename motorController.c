#include "IO_Driver.h"
#include "IO_RTC.h"
#include "motorController.h"
#include "mathFunctions.h"
#include "sensors.h"
#include "readyToDriveSound.h"

//Import extrenal variables
//extern Sensor Sensor_WPS_FR; // = { 3 };

extern MotorController MCU0;
//Update the MCU object from its CAN messages
void motorController_setCommands(ReadyToDriveSound* rtds) 
{
    //Temp hardcode
    MCU0.commands.setDischarge = DISABLED;

    //1 = forwards for our car, 0 = reverse
    MCU0.commands.direction = 1;

    //Set Torque/Inverter control
    if (Sensor_WPS_FL.fresh == FALSE)
    {
        if (MCU0.commands.requestedTorque != 0) MCU0.commands.updateCount++;
        MCU0.commands.requestedTorque = 0;
    }
    else
    {
        ubyte2 torqueSetting;  //temp variable to store torque calculation
        //CURRENTLY: Don't command torque until >1s after the inverter is enabled, otherwise CAN breaks
        if (IO_RTC_GetTimeUS(MCU0.timeStamp_inverterEnabled) <= 1000000)
        {
            torqueSetting = 0;
            if (MCU0.commands.requestedTorque != torqueSetting) MCU0.commands.updateCount++;
        }
        else
        {
            torqueSetting = 100 * getPercent(Sensor_WPS_FL.sensorValue, 500, 2500, TRUE); //int is fine
            if (MCU0.commands.requestedTorque != torqueSetting) MCU0.commands.updateCount++;
            MCU0.commands.requestedTorque = torqueSetting;
        }
    }

    //New Handshake NOTE: Switches connected to ground.. TRUE = high = off = disconnected = open circuit, FALSE = low = grounded = on = connected = closed circuit
    if (MCU0.lockoutStatus == ENABLED)
    {
        //If not on gas and YES on break and RTD is pressed
        if (Sensor_WPS_FL.sensorValue < 10 && Sensor_TEMP_BrakingSwitch.sensorValue == FALSE && Sensor_RTD_Button.sensorValue == FALSE)
        {
            //Begin the startup sequence, but don't start the RTDS yet
            MCU0.commands.setInverter = DISABLED;
            MCU0.startRTDS = FALSE;
        }
    }
    else  //Lockout has already been disabled
    {
        if (MCU0.inverterStatus == DISABLED)
        {
            MCU0.commands.setInverter = ENABLED;
            IO_RTC_StartTime(&MCU0.timeStamp_inverterEnabled);
            MCU0.startRTDS = TRUE;
        }
        else  
        {
            //If the inverter was successfully enabled AND we haven't started the RTDS yet
            if (MCU0.startRTDS == TRUE)
            {
                RTDS_setVolume(rtds, .005, 1500000);
                MCU0.startRTDS = FALSE;                
            }
        }
    }
}