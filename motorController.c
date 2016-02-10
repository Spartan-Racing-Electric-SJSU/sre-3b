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
        if (IO_RTC_GetTimeUS(MCU0.commands.timeStamp_inverterEnabled) <= 1000000)
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
    if (MCU0.lockoutStatus == ENABLED && Sensor_WPS_FL.sensorValue < 10 && Sensor_TEMP_BrakingSwitch.sensorValue == FALSE && Sensor_RTD_Button.sensorValue == FALSE)
    {
        MCU0.commands.setInverter = DISABLED;
        RTDS_setVolume(rtds, .005, 15000);
    }
    else
    {
        if (MCU0.commands.setInverter == DISABLED)
        {
            MCU0.commands.setInverter = ENABLED;
            IO_RTC_StartTime(&MCU0.commands.timeStamp_inverterEnabled);
        }
    }

    /*Old handshake
    if (Sensor_WPS_FL.sensorValue < 10 && MCU0.lockoutStatus == ENABLED)
    {
        MCU0.commands.setInverter = DISABLED;
    }
    else
    {
        if (MCU0.commands.setInverter == DISABLED)
        {
            MCU0.commands.setInverter = ENABLED;
            IO_RTC_StartTime(&MCU0.commands.timeStamp_inverterEnabled);
            //MCU0.commands.inverterHasBeenEnableed = TRUE;
        }
    }*/
}