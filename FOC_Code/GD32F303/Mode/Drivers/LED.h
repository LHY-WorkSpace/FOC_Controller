#ifndef LED_H
#define LED_H

#define LED_OFF      gpio_bit_set(GPIOA,GPIO_PIN_1)
#define LED_ON       gpio_bit_reset(GPIOA,GPIO_PIN_1)

void LED_Init(void);

#endif







