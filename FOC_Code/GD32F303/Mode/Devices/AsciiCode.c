#include "DataType.h"
#include "AsciiCode.h"
#include "gd32f30x.h"
#include "string.h"
#include "LED.h"

// static char AsciiBuff[1024];
// static u16 TotalData = 0;

// void AsciiCode_Init(void)
// {
// 	TotalData = 0;
// 	AsciiCodeSend("123");
// 	// MorseCodeSend("JACK TRUST ME");
// }


// void AsciiCodeSend(char *Data)
// {
// 	TotalData = strlen(Data);
// 	memcpy(AsciiBuff,Data,TotalData);
// }


// 10ms调用一次
void AsciiCodeTimerTick()
{
	// static u8 Sft = 0; 
	// static u16 BufCnt = 0;

	// if(TotalData == 0)
	// {
	// 	return;
	// }

	// if( Sft < 8 )
	// {
	// 	if((0x80>>Sft)&AsciiBuff[BufCnt])
	// 	{
	// 		LED_OFF;//高电平
	// 	}
	// 	else
	// 	{
	// 		LED_ON;//低电平
	// 	}
	// 	Sft++;
	// }
	// else
	// {
	// 	Sft++;
	// 	if(Sft >= 16)
	// 	{
	// 		Sft = 0;
	// 		BufCnt++;
	// 		if( (BufCnt >= sizeof(AsciiBuff) )|| BufCnt >= TotalData)
	// 		{
	// 			BufCnt = 0;
	// 			TotalData = 0;
	// 		}
	// 	}
	// 	LED_ON;//低电平
	// }

}













































