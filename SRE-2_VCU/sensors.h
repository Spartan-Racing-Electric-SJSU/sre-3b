
//----------------------------------------------------------------------------
// Sensor Object Definitions
//----------------------------------------------------------------------------
// Parameters:
// canMessageID - REQUIRED - Unique ID (address) for each CAN message.  Multiple sensors
//                can be assigned to a single ID/message if they'll fit in the
//                available data space
//                TODO: What's the limit?
//				  https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit
// TODO: Write the rest of these
//
// specMin/Max values should come from each sensor's datasheets, but it is not
// required for all sensors.
//
// TODO: What about having default calbiration values?  (Probably useless)
//----------------------------------------------------------------------------
typedef struct _Sensor {
    //CAN stuff
    ubyte1 canMessageID;  //Required!

                          //Sensor values / properties
    ubyte2 specMin;
    ubyte2 specMax;
    bool isCalibrated;
    ubyte2 calibMin;
    ubyte2 calibMax;
    ubyte2 calibNormal;  //zero value or normal position

    ubyte2 sensorValue;
    bool fresh;

    ubyte2 calibratedValue;

} Sensor;



//----------------------------------------------------------------------------
// Sensor Object Instantiations
//----------------------------------------------------------------------------
// canMessageID should be specified NOW!
//----------------------------------------------------------------------------

//TODO: Read stored calibration data from EEPROM

//Torque Encoders (TPS is not really accurate since there's no throttle to position in an EV)
Sensor Sensor_TPS0 = { 0, 0.5, 4.5 };
Sensor Sensor_TPS1 = { 0, 4.5, 0.5 };

//Brake Position Sensors
Sensor Sensor_BPS0 = { 1, 0.5, 4.5 };  //Brake system pressure (or front only in the future)
                                       //Sensor Sensor_BPS1 = { 2, 0.5, 4.5 }; //Rear brake system pressure (separate address in case used for something else)

                                       //Wheel Speed Sensors (like an ABS sensor)
Sensor Sensor_WSS_FL = { 3 };
Sensor Sensor_WSS_FR = { 3 };
Sensor Sensor_WSS_RL = { 3 };
Sensor Sensor_WSS_RR = { 3 };

//Wheel Position Sensors (Shock pots)
Sensor Sensor_WPS_FL = { 4 };
Sensor Sensor_WPS_FR = { 4 };
Sensor Sensor_WPS_RL = { 4 };
Sensor Sensor_WPS_RR = { 4 };

//Switches
//precharge failure

//Other
Sensor Sensor_LVBattery = { 5 };  //Note: There will be no init for this "sensor"