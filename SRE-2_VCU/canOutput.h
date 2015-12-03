#ifndef _CANOUTPUT_H
#define _CANOUTPUT_H




// canMessageBaseId - The starting CAN address for all messages
// Note: CAN message address = canMessageBaseId + canMessageIdOffset
const ubyte1 canMessageBaseId = 0x500;

IO_CAN_DATA_FRAME canSensorMessages[10] = { { { 0 } } };

const ubyte1 canSpeed_Channel0 = 500;
const ubyte1 canSpeed_Channel1 = 250;

// canMessageIdOffset - Unique ID (address) for each CAN message, starting at
//                      CANBaseAddress.  Multiple sensors
//                can be assigned to a single ID/message if they'll fit in the
//                available data space
//                TODO: What's the limit?
//				  https://docs.google.com/spreadsheets/d/1sYXx191RtMq5Vp5PbPsq3BziWvESF9arZhEjYUMFO3Y/edit


//Functions
void broadcastSensorCanMessages(void);

#endif //  _CANOUTPUT_H