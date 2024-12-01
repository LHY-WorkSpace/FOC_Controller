#ifndef  WS2812_H
#define  WS2812_H



#define  WS2812_NUM     (30)


void  WS2812_Init(void);
void  WS2812_SetColor(u8 Red, u8 Green, u8 Blue,u8 Num);
void  RGB_SendToLED(void);
void  AsciiCodeSend(char *Data);
void  WS2812_SetAll(u8 Red, u8 Green, u8 Blue);
#endif



