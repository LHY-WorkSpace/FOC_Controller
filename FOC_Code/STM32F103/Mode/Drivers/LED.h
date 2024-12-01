#ifndef LED_H
#define LED_H


#define LED_OFF      GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define LED_ON       GPIO_SetBits(GPIOB,GPIO_Pin_0)


#define LED_ON_R      GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define LED_OFF_R     GPIO_SetBits(GPIOB,GPIO_Pin_14)

void LED_Init(void);

#endif







