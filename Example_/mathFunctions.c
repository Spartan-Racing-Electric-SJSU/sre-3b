#include <stdarg.h>
#include <math.h>
#include "mathFunctions.h"
#include "IO_Driver.h"
#include "IO_RTC.h"

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* Automatically compensates for reverse-direction values: When min > max, value is assumed to travel in reverse direction.
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/

float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly)
{
    float4 retVal;
    
    //Flip values and start at 0
    float4 min = (start < end) ? start : end;
    float4 max = (start < end) ? end : start;
    //float4 offset = (start < end) ? value - start : end - value; //fabs(max - value);
    
    float4 range = fabs(start - end); //(start < end) ? end : start;
    float4 offset = (start < end) ? value - start : end - value; //fabs(max - value);

    if (zeroToOneOnly == TRUE)
    {
/*            if (value >= max) { retVal = 1; }
            else if (value <= min) { retVal = 0; }
            else { retVal = offset / (max - min); */
        if (offset < 0) { retVal = 0; }
        else if (offset > range) { retVal = 1; }
        else { retVal = offset / range; }
    }
    else
    {
        retVal = (offset) / range; //(max - min);
    }

    return retVal;
}


// A utility function to get maximum of two integers
ubyte2 max(ubyte2 a, ubyte2 b)
{
    return (a > b) ? a : b;
}

// A utility function to get maximum of two integers
ubyte2 min(ubyte2 a, ubyte2 b)
{
    return (a < b) ? a : b;
}


/**********************************************************************//**
 *
 * \brief Decides whether a blinking light should be on or off based on
 *        the system clock and period given
 *
 * \param timestamp     A timestamp from IO_RTC for when the blink was started
 * \param highPeriod    How long the blink should stay high, in ms (NOT ns)
 * \param lowPeriod     How long the blink should stay low -- NOT IMPLEMENTED;
 *                      Always the same as highPeriod
 *
 * \return boolean:
 * \retval TRUE      blink should be high
 * \retval FALSE     blink should be low
 *
 ***************************************************************************
 *
 * \remarks
 *   Blablabla
 *
 ***************************************************************************/
bool blink(ubyte4* clock, ubyte2 highPeriod)
{
    //time passed since the start of the blinks divided by the period to get
    //count.
    //count % 2 gets the current state the blink should be in 0 or <0
    ubyte4 count = IO_RTC_GetTimeUS(*clock) / highPeriod;
    //removes decimal places. there may be a better way to do this but I got lazy
    count = count - count % 1;

    return !( count / highPeriod ) % 2;
}





//byte swapping functions used by BMS

ubyte1 swap_uint8(ubyte1 val)
{
    return (val << 4) | (val >> 4);
}

sbyte1 swap_int8(sbyte1 val)
{
    return (val << 4) | (val >> 4);
}
ubyte2 swap_uint16(ubyte2 val)
{
    return (val << 8) | (val >> 8);
}

//! Byte swap short
sbyte2 swap_int16(sbyte2 val)
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned int
ubyte4 swap_uint32(ubyte4 val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | (val >> 16);
}

//! Byte swap int
sbyte4 swap_int32(sbyte4 val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

