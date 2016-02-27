
#include <math.h>
#include "mathFunctions.h"
#include "IO_Driver.h"

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly)
{
    float4 retVal;

    //First of all, flip the start/end around if the values go backwards
    float4 min = start; //(start < end) ? start : end;
    float4 max = end; //(start < end) ? end : start;
    float4 val = value; //fabs(max - value);

    if (zeroToOneOnly == TRUE)
    {
            if (value >= max) { retVal = 1; }
            else if (value <= min) { retVal = 0; }
            else { retVal = (value - min) / (max - min); }
    }
    else
    {
        retVal = (value - min) / (max - min);
    }

    return retVal;
}
