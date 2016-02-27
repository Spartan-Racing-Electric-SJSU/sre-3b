#ifndef _MATHFUNCTIONS_H
#define _MATHFUNCTIONS_H


#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file

/*****************************************************************************
* Helper functions
****************************************************************************/
/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
float4 getPercent(float4 value, float4 start, float4 end, bool zeroToOneOnly);



#endif //  _MATHFUNCTIONS_H
