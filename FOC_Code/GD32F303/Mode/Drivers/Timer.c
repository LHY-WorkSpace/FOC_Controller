
#include "Timer.h"
#include "DataType.h"
#include "gd32f30x.h"
#include "Key.h"
#include "MorseCode.h"
#include "AsciiCode.h"
// Timer1-4

#define MAX_VAL (1000) // 10 S

static u32 Count;
static u8 Time_Flag;

// 10 ms
void TIMER_Init(u16 TickVal)
{
	timer_parameter_struct timer_initpara;

	rcu_periph_clock_enable(RCU_TIMER1);

	/* TIMER1  configuration */
	timer_deinit(TIMER1);

	timer_initpara.prescaler         = 120 - 1;
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = TickVal*1000 - 1;
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;

	timer_auto_reload_shadow_enable(TIMER1);
	timer_interrupt_enable(TIMER1,TIMER_INT_UP);

	timer_init(TIMER1,&timer_initpara);
	Time_Flag  = 0;
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
void TIMER1_IRQHandler(void) // TIM3中断
{
	if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) != RESET)
	{
		timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);


	}
}
