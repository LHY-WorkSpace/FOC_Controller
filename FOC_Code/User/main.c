#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"
#include "u8g2_UserGUI.h"
#include "ADC.h"
#include "WS2812.h"
#include "CAN.h"

int main(void)
{
  u8 i;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  Delay_Init();
  LED_Init();
  // Key_Init();
  // // EEPROM_Init();
  //   Delay_ms(100);
  AS5600_Init();
  PWM_Init();
  USART1_Init(115200);
  // u8g2_Init();

  // AD_Init();
  // Can_Init(0,0,0,0,0);


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

