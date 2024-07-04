#ifndef CAN_H
#define CAN_H

#define CAN_CLK_DIV     (10)


// #define BUFFER_SIZE      (1024)

// typedef struct 
// {
//     u16 RX_Pointer;//已接收数据指针
//     u16 TX_Pointer;//已发送数据指针
//     u16 RX_Length;//已接收的数据长度
//     u16 TX_Length;//要发送的数据长度
//     u8 RX_Data[BUFFER_SIZE]; 
//     u8 TX_Data[BUFFER_SIZE];
// }USART_Data_t;


// extern USART_Data_t USART1_Data;
// extern USART_Data_t USART2_Data;

// void CAN_Init();  
void Can_Init(u8 tsjw,u8 tbs1,u8 tbs2,u16 brp,u8 mode); 
u8 Med_Can_Receive_Msg (u8 *buf);
u8 Med_Can_Send_Msg (u8* msg,u8 len);
// void USART2_Init(u32 bode,u16 DataLength,u16 StopBit,u16 Parity);
// void USARTx_ITHandle(USART_TypeDef* USARTx,USART_Data_t *USART_Data);
// u8 USART_ITSendData(USART_TypeDef* USARTx,USART_Data_t *USART_Data,u16 Length,u8 *Data);
// u8 USART_GetData(USART_Data_t *USART_Data,u16 Buffsize,u8 *Data,u16 *Length);
// void USART_PollingSendData(USART_TypeDef* USARTx,USART_Data_t *USART_Data,u8 *Data,u16 Length);
// int fputc(int ch, FILE* stream);
// u8 *USART_RxDataAddr(USART_Data_t *USART_Data,u16 offset);
// u8 *USART_TxDataAddr(USART_Data_t *USART_Data,u16 offset);
#endif







