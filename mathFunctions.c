
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
float4 getPercent(float4 value, float4 min, float4 max, bool zeroToOneOnly)
{
    float4 retVal;
    if (value >= max) { retVal = 1; }
    else if (value <= min) { retVal = 0; }
    else { retVal = (value - min) / (max - min); }

    return retVal;
}

float4 getPercentOLD(float4 value, float4 min, float4 max, bool zeroToOneOnly)
{
    float4 retVal = (value - min) / (max - min);

    if (zeroToOneOnly == TRUE)
    {
        if (value > max) { retVal = 1; }
        if (value < min) { retVal = 0; }
    }

    return retVal;
}
