
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
