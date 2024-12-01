#ifndef TIMER_H
#define TIMER_H
#include "stm32f10x.h"

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

void TIM3_Init(u16 arr, u16 psc);
void Time_SetFlag( u8 Flag);
void Time_ResetFlag( u8 Flag);
u8 Time_GetFlag( u8 Flag);
void Time_SetAllFlag(u8 Type);

void Delay_Init(void);
void Delay_us(u16 nus);
void Delay_ms(u16 nus);
u32 Time_GetInterval(u32 *LastTick);

#endif
