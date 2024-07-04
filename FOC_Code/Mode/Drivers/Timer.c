
#include "Timer.h"
#include "DataType.h"
#include "stm32f10x.h"
#include "Key.h"
#include "MorseCode.h"
#include "AsciiCode.h"
// Timer1-4

#define MAX_VAL (1000) // 10 S

static u32 Count;
static u8 Time_Flag;

// 10 ms
void TIM3_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// 定时器TIM3初始化
	TIM_TimeBaseStructure.TIM_Period = arr *1000 - 1;// N ms
	TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	// 中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM3, ENABLE);
	Time_Flag  = 0;
}

void Delay_Init()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStr;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
	TIM_TimeBaseInitStr.TIM_Period = 0Xffff;
	TIM_TimeBaseInitStr.TIM_Prescaler = 72-1;//1us
	TIM_TimeBaseInitStr.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStr.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStr);
    TIM_ARRPreloadConfig(TIM4,DISABLE);
}

void Delay_us(u16 nus)
{
    TIM4->CNT = 0;
    TIM_Cmd(TIM4,ENABLE);
    while (TIM4->CNT < nus);
    TIM_Cmd(TIM4,DISABLE);
}

void Delay_ms(u16 nus)
{
    u16 i; 
    for(i=0;i<nus;i++)
    {
        TIM4->CNT = 0;
        TIM_Cmd(TIM4,ENABLE);
        while (TIM4->CNT < 1000); 
        TIM_Cmd(TIM4,DISABLE);
    }
}


void Time_SetFlag(u8 Flag)
{
	Time_Flag |= (1 << Flag);
}

void Time_ResetFlag(u8 Flag)
{
	Time_Flag &= ~(1 << Flag);
}

u8 Time_GetFlag(u8 Flag)
{
	if( (Time_Flag & (1 << Flag)) )
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}


// 获取调用间隔
// LastTick必须为 static 类型
u32 Time_GetInterval(u32 *LastTick)
{
	u32 Interval = 0;
	if(Count < *LastTick)
	{
		Interval = 0xFFFFFFFF - *LastTick + Count;
	}
	else
	{
		Interval = Count - *LastTick;
	}

	*LastTick = Count;

	return Interval;
}


// 定时器3中断服务程序  10ms
void TIM3_IRQHandler(void) // TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		// AsciiCodeTimerTick();
		MorseCodeTimerTick();
		button_ticks();

		if (Count >= MAX_VAL)
		{
			Count = 0;
		}
		Time_SetFlag(Flag_10ms);

		if (Count % 2 == 0)
		{
			Time_SetFlag(Flag_20ms);
		}

		if (Count % 3 == 0)
		{
			Time_SetFlag(Flag_30ms);
		}

		if (Count % 5 == 0)
		{
			Time_SetFlag(Flag_50ms);
		}

		if (Count % 10 == 0)
		{
			Time_SetFlag(Flag_100ms);
		}

		if (Count % 20 == 0)
		{
			Time_SetFlag(Flag_200ms);
		}

		if (Count % 50 == 0)
		{
			Time_SetFlag(Flag_500ms);
		}
		
		Count++;
	}
}
