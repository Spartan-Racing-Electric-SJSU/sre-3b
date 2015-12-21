//"Include guard" - prevents this file from being #included more than once
#ifndef _READYTODRIVESOUND_H
#define _READYTODRIVESOUND_H

struct _ReadyToDriveSound;

typedef ReadyToDriveSound struct _ReadyToDriveSound;

ReadyToDriveSound* RTDS_new(char* rtds);
void RTDS_delete(ReadyToDriveSound* rtds);

void RTDS_setVolume(ReadyToDriveSound* rtds, float4 volumePercent);


#endif // _READYTODRIVESOUND_H
