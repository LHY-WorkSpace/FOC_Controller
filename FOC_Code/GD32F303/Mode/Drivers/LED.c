#include "gd32f30x.h"
#include "DataType.h"
#include "LED.h"
 



void LED_Init()
{
    rcu_periph_clock_enable(RCU_GPIOA); 
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
}

