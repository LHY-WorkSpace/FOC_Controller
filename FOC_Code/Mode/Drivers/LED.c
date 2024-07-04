#include "stm32f10x.h"
#include "DataType.h"
#include "LED.h"
 



void LED_Init()
{

	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	LED_OFF;
	LED_OFF_R;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_14;	
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_Initstructure);


}