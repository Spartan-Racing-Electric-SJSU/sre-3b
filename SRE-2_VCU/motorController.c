
void main(void)
{

    //Remove lockout by sending Inverter disable command

    //

    //Rinehart CAN control message (heartbeat) structure ----------------
    IO_CAN_DATA_FRAME canMCUControlMessage;
    canMCUControlMessage.length = 8; // how many bytes in the message
    canMCUControlMessage.id_format = IO_CAN_STD_FRAME;
    canMCUControlMessage.id = 0xC0;

    //Torque (Nm * 10)
    ubyte2 mcuTorque = 5; //In Nm * 10. 125 continuous, 240 max
    canMCUControlMessage.data[0] = (ubyte1)mcuTorque;
    canMCUControlMessage.data[1] = mcuTorque >> 8;

    //Speed (RPM?) - not needed - mcu should be in torque mode
    canMCUControlMessage.data[2] = 0;
    canMCUControlMessage.data[3] = 0;

    //Direction
    canMCUControlMessage.data[4] = 1;

    //unused/unused/unused/unused unused/unused/Discharge/Inverter Enable
    canMCUControlMessage.data[5] = 0b00000001;

    //Torque Limit?
    canMCUControlMessage.data[2] = 0;
    canMCUControlMessage.data[3] = 0;



}