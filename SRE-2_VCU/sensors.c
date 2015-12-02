void main()
{

    //Create CAN Message Objects ---------------------------------------------------
    const ubyte1 canMessageCount = 1;
    IO_CAN_DATA_FRAME canMessage[canMessageCount] = { { { 0 } } };

    for (ubyte1 i = 0x500; i < canMessageCount; i++)
    {
        canMessage[i].id = i;
        canMessage[i].id_format = IO_CAN_STD_FRAME;
        canMessage[i].length = 8;
    }

    //Activate the CAN channels ---------------------------------------------------
    ubyte1 handle_fifo_w; //What is this for?
    IO_CAN_Init(IO_CAN_CHANNEL_0, 250, 0, 0, 0);
    IO_CAN_ConfigFIFO(&handle_fifo_w, IO_CAN_CHANNEL_0, canMessageCount, IO_CAN_MSG_WRITE, IO_CAN_STD_FRAME, 0, 0);



    ubyte4 timestamp_sensorpoll = 0;
    while (1)
    {
        //TPS CAN messages
        canMessage[Sensor_TPS0.canMessageID].length = 4;                             //Little endian
        canMessage[Sensor_TPS0.canMessageID].data[0] = Sensor_TPS0.sensorValue;      //TPS0.lowbyte
        canMessage[Sensor_TPS0.canMessageID].data[1] = Sensor_TPS0.sensorValue >> 8; //TPS0.hibyte
        canMessage[Sensor_TPS0.canMessageID].data[2] = Sensor_TPS1.sensorValue;      //TPS0.lowbyte
        canMessage[Sensor_TPS0.canMessageID].data[3] = Sensor_TPS1.sensorValue >> 8; //TPS0.hibyte
                                                                                     //Load data into CAN
        canMessage[Sensor_BPS0.canMessageID].length = 2;
        canMessage[Sensor_BPS0.canMessageID].data[0] = Sensor_BPS0.sensorValue;
        canMessage[Sensor_BPS0.canMessageID].data[1] = Sensor_BPS0.sensorValue >> 8;


        canMessage[Sensor_WPS_FL.canMessageID].length = 8;
        canMessage[Sensor_WPS_FL.canMessageID].data[0] = (ubyte1)Sensor_WPS_FL.sensorValue;
        canMessage[Sensor_WPS_FL.canMessageID].data[1] = Sensor_WPS_FL.sensorValue >> 8;
        canMessage[Sensor_WPS_FR.canMessageID].data[2] = (ubyte1)Sensor_WPS_FR.sensorValue;
        canMessage[Sensor_WPS_FR.canMessageID].data[3] = Sensor_WPS_FR.sensorValue >> 8;
        canMessage[Sensor_WPS_RL.canMessageID].data[4] = (ubyte1)Sensor_WPS_RL.sensorValue;
        canMessage[Sensor_WPS_RL.canMessageID].data[5] = Sensor_WPS_RL.sensorValue >> 8;
        canMessage[Sensor_WPS_RR.canMessageID].data[6] = (ubyte1)Sensor_WPS_RR.sensorValue;
        canMessage[Sensor_WPS_RR.canMessageID].data[7] = Sensor_WPS_RR.sensorValue >> 8;


        canMessage[Sensor_WSS_FL.canMessageID].length = 8;
        canMessage[Sensor_WSS_FL.canMessageID].data[0] = Sensor_WSS_FL.sensorValue;
        canMessage[Sensor_WSS_FL.canMessageID].data[1] = Sensor_WSS_FL.sensorValue >> 8;
        canMessage[Sensor_WSS_FL.canMessageID].data[2] = Sensor_WSS_FR.sensorValue;
        canMessage[Sensor_WSS_FL.canMessageID].data[3] = Sensor_WSS_FR.sensorValue >> 8;
        canMessage[Sensor_WSS_FL.canMessageID].data[4] = Sensor_WSS_RL.sensorValue;
        canMessage[Sensor_WSS_FL.canMessageID].data[5] = Sensor_WSS_RL.sensorValue >> 8;
        canMessage[Sensor_WSS_FL.canMessageID].data[6] = Sensor_WSS_RR.sensorValue;
        canMessage[Sensor_WSS_FL.canMessageID].data[7] = Sensor_WSS_RR.sensorValue >> 8;

        canMessage[Sensor_LVBattery.canMessageID].length = 2;
        canMessage[Sensor_LVBattery.canMessageID].data[0] = Sensor_LVBattery.sensorValue;
        canMessage[Sensor_LVBattery.canMessageID].data[1] = Sensor_LVBattery.sensorValue >> 8;


        //----------------------------------------------------------------------------
        // CAN
        //----------------------------------------------------------------------------
        IO_CAN_WriteFIFO(handle_fifo_w, canMessage, canMessageCount);

}