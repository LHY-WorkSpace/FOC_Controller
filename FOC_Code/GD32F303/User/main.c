#include "gd32f30x.h"
#include "main.h"
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"
#include "LED.h"
#include "usart.h"

int main(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    systick_config();
    LED_Init();
    // // EEPROM_Init();
    Delay_ms(100);
    AS5600_Init();
    PWM_Init();
    USART1_Init(115200);


    while (1)
    {
        Foc_CTL();
        // u8g2_Task();
        // WS2812_SetColor(10,10,240,1);
        // RGB_SendToLED();
        // Med_Can_Send_Msg(Bufftest,5);
        // Delay_ms(500);
        // Task();
    }


}

