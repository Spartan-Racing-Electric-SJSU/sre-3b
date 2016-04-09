#ifndef _CANINPUT_H
#define _CANINPUT_H

#include "can.h"
#include "motorController.h"

void canInput_readMessages(MotorController* mcm);
ubyte1 swap_uint8(ubyte1 val);
sbyte1 swap_int8(sbyte1 val);
ubyte2 swap_uint16(ubyte2 val);
sbyte2 swap_int16(sbyte2 val);
ubyte4 swap_uint32(ubyte4 val);
sbyte4 swap_int32(sbyte4 val);


#endif //  _CANINPUT_H
