#ifndef TIMER_H
#define TIMER_H
#include "gd32f30x.h"
#include "DataType.h"

typedef enum
{
    Flag_10ms,
    Flag_20ms,
    Flag_30ms,
    Flag_50ms,
    Flag_100ms,
    Flag_200ms,
    Flag_500ms,
    Flag_Max,
} Flag_e;

void TIMER_Init(u16 TickVal);
void Time_SetFlag( u8 Flag);
void Time_ResetFlag( u8 Flag);
u8 Time_GetFlag( u8 Flag);
void Time_SetAllFlag(u8 Type);

u32 Time_GetInterval(u32 *LastTick);

#endif
