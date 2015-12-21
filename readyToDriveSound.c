#include <stdlib.h>  //Needed for malloc

#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_PWM.h"

#include "readyToDriveSound.h"


struct _ReadyToDriveSound
{
    ubyte4 timeStamp_soundStarted;  //from IO_RTC_StartTime(&)
    ubyte2 timeToSound;
    ubyte2 volumePercent;
};

ReadyToDriveSound* RTDS_new(void)
{
    ReadyToDriveSound* rtds = (ReadyToDriveSound*)malloc(sizeof(struct _ReadyToDriveSound));
    RTDS_setVolume(rtds, 0)
    return rtds;
}

void RTDS_delete(ReadyToDriveSound* rtds)
{
    // some implementations pass a Person** to set the reference to 0
    // this implementation requires that the caller sets his own references to 0
    //free(person->name);
    free(rtds);
}

void RTDS_setVolume(ReadyToDriveSound* rtds, float4 volumePercent)
{
    IO_PWM_SetDuty(IO_PWM_07, 65535 * volumePercent, NULL);  //Pin 103
}