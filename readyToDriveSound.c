#include "readyToDriveSound.h"

struct _ReadyToDriveSound
{
    ubyte4 timeStamp_soundStarted;  //from IO_RTC_StartTime(&)
    ubyte2 timeToSound;
    ubyte2 volumePercent;
};

ReadyToDriveSound* RTDS_new()
{
    ReadyToDriveSound* rtds = (ReadyToDriveSound*)malloc(sizeof(struct _ReadyToDriveSound));
    return rtds;
}

void RTDS_delete(ReadyToDriveSound* rtds) {
    // some implementations pass a Person** to set the reference to 0
    // this implementation requires that the caller sets his own references to 0
    //free(person->name);
    free(ReadyToDriveSound);
}

void RTDS_setVolume(ReadyToDriveSound* rtds, float4 volumePercent)
{

}