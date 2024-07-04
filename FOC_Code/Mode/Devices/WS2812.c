#include "DataType.h"
#include "WS2812.h"
#include "stm32f10x.h"


// ���1��Ϊ��λ֡
static u16 DispBuff[24*(WS2812_NUM+1)];

const u16 Code0 = 24;
const u16 Code1 = 66;

static u8 ComplateFlag = 1;

static u16 TotalData = 0;


// PA2  TIM2_CH3
void  WS2812_Init(void)
{
	u16 i;
	TIM_TimeBaseInitTypeDef          TIM_TimeBaseStructure;
	TIM_OCInitTypeDef                TIM_OCInitStructure;
	GPIO_InitTypeDef                 GPIO_InitStructure;
	DMA_InitTypeDef                  DMA_InitStructure;
	NVIC_InitTypeDef 				 NVIC_Initstr;


	//clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	//GPIO
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//TIME
	TIM_TimeBaseStructure.TIM_Period = 90-1; // 800kHZ  72/80=800KHZ
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	//PWM
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;//�߼���ʱ�����һ��Ҫ��
	TIM_OCInitStructure.TIM_Pulse = 0; //CCRX�Ĵ���ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);//ʹ��Ԥ����ֵ�����һ��Ҫ��,��Ȼʱ�����
	TIM_ARRPreloadConfig(TIM2,ENABLE);


	NVIC_Initstr.NVIC_IRQChannel=DMA1_Channel2_IRQn;
	NVIC_Initstr.NVIC_IRQChannelPreemptionPriority=4;
	NVIC_Initstr.NVIC_IRQChannelSubPriority=0;
	NVIC_Initstr.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Initstr);

	//DMA
	// DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(TIM2->CCR3);//����CCRֵ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&DispBuff;	//���CCRֵ������
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���ﴫ����Ƕ���λ�������㶨��Ĵ��CCRֵ���������ͣ�����u16
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���ﴫ����Ƕ���λ�������㶨��Ĵ��CCRֵ���������ͣ�����u16
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
	TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
	TIM_Cmd(TIM2, DISABLE);

	for (i = 0; i < 24*WS2812_NUM; i++)
	{
		DispBuff[i] = 00;
	}
	RGB_SendToLED();
	Delay_ms(500);
}




// Num:�ڼ����ƣ���0��ʼ
void  WS2812_SetColor(u8 Red, u8 Green, u8 Blue,u8 Num)
{
	u8 i;

	for(i = 0; i < 8; i++)
	{
		if( (Green&(0x80>>i)) )
		{
			DispBuff[Num*24+i] = Code1;
		}
		else
		{
			DispBuff[Num*24+i] = Code0;
		}

		if( (Red&(0x80>>i)) )
		{
			DispBuff[Num*24+i+8] = Code1;
		}
		else
		{
			DispBuff[Num*24+i+8] = Code0;
		}

		if( (Blue&(0x80>>i)) )
		{
			DispBuff[Num*24+i+16] = Code1;
		}
		else
		{
			DispBuff[Num*24+i+16] = Code0;
		}
	}
	memset(&DispBuff[(Num+1)*24],0x00,3);
}


void  WS2812_SetAll(u8 Red, u8 Green, u8 Blue)
{
	u8 i;

	for(i = 0; i < WS2812_NUM; i++)
	{
		WS2812_SetColor(Red,Green,Blue,i);
	}
}



// void AsciiCodeSend(char *Data)
// {
// 	u16 i,k;
// 	char *Pbuf = Data;
// 	TotalData = strlen(Data);
// 	for (i = 0; i < TotalData; i++)
// 	{
// 		for (k = 0; k < 8; k++)
// 		{
// 			if( ((*Data)&(0x80>>k)) )
// 			{
// 				DispBuff[i*8+k] = Code1;
// 			}
// 			else
// 			{
// 				DispBuff[i*8+k] = Code0;
// 			}
// 		}
// 		Pbuf++;
// 	}
// }







void RGB_SendToLED()
{
	if(ComplateFlag == 1)
	{
		DMA_SetCurrDataCounter(DMA1_Channel2,24*(WS2812_NUM+1));
		DMA_Cmd(DMA1_Channel2,ENABLE);
		TIM_Cmd(TIM2,ENABLE);
		ComplateFlag = 0;
	}
}


void DMA1_Channel2_IRQHandler()
{
	DMA_ClearITPendingBit(DMA1_IT_TC2);
	DMA_ClearFlag(DMA1_FLAG_TC2);
	ComplateFlag = 1;
	TIM_Cmd(TIM2,DISABLE);//��ʱ�������������
	DMA_Cmd(DMA1_Channel2,DISABLE);
}





















	NVIC_InitTypeDef  NVIC_Initstr;


	// NVIC_Initstr.NVIC_IRQChannel=USART1_IRQn;
	// NVIC_Initstr.NVIC_IRQChannelPreemptionPriority=4;
	// NVIC_Initstr.NVIC_IRQChannelSubPriority=0;
	// NVIC_Initstr.NVIC_IRQChannelCmd=ENABLE;
	// NVIC_Init(&NVIC_Initstr);



















































