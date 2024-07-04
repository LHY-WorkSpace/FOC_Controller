#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "CAN.h"


// USART_Data_t USART1_Data;
// USART_Data_t USART2_Data;

/*
PA11 - CAN_RX == IC RX
PA12 - CAN_TX == IC TX
*/
// void CAN_Init()
// {
// 	GPIO_InitTypeDef USART_GPIO_Init;
// 	CAN_InitTypeDef CAN_InitType;
// 	NVIC_InitTypeDef  NVIC_Initstr;
// 	CAN_FilterInitTypeDef CAN_FilterInit;

// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

// 	USART_GPIO_Init.GPIO_Pin = GPIO_Pin_11;
// 	USART_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
// 	USART_GPIO_Init.GPIO_Mode = GPIO_Mode_AF_PP;
// 	GPIO_Init(GPIOA,&USART_GPIO_Init);

// 	USART_GPIO_Init.GPIO_Pin = GPIO_Pin_12;
// 	USART_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
// 	USART_GPIO_Init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
// 	GPIO_Init(GPIOA,&USART_GPIO_Init);

// 	CAN_InitType.CAN_Prescaler = CAN_CLK_DIV;
// 	CAN_InitType.CAN_Mode = CAN_Mode_Normal;
// 	CAN_InitType.CAN_SJW = CAN_SJW_1tq;
// 	CAN_InitType.CAN_BS1 = CAN_BS1_8tq;
// 	CAN_InitType.CAN_BS2 = CAN_BS2_8tq;
// 	CAN_InitType.CAN_TTCM = DISABLE;
// 	CAN_InitType.CAN_ABOM = DISABLE;
// 	CAN_InitType.CAN_AWUM = DISABLE;
// 	CAN_InitType.CAN_NART = DISABLE;
// 	CAN_InitType.CAN_RFLM = DISABLE;
// 	CAN_InitType.CAN_TXFP = DISABLE;
// 	CAN_Init(CAN1,&CAN_InitType);

// 	CAN_FilterInit.CAN_FilterIdHigh = ;
// 	CAN_FilterInit.CAN_FilterIdLow = ;
// 	CAN_FilterInit.CAN_FilterMaskIdHigh = ;
// 	CAN_FilterInit.CAN_FilterMaskIdLow = ;	
// 	CAN_FilterInit.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;
// 	CAN_FilterInit.CAN_FilterNumber = ;
// 	CAN_FilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
// 	CAN_FilterInit.CAN_FilterScale = CAN_FilterScale_16bit;	
// 	CAN_FilterInit.CAN_FilterActivation = DISABLE;
// 	CAN_FilterInit(&CAN_FilterInit);

// 	NVIC_Initstr.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
// 	NVIC_Initstr.NVIC_IRQChannelPreemptionPriority = 4;
// 	NVIC_Initstr.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_Initstr.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_Initstr);

// 	NVIC_Initstr.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
// 	NVIC_Initstr.NVIC_IRQChannelPreemptionPriority = 4;
// 	NVIC_Initstr.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_Initstr.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_Initstr);

// 	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
// 	CAN_ClearFlag(CAN1,0x3ff);

// 	CAN_WakeUp(CAN1);
// }


// void CanTX()
// {
// 	CanTxMsg TxData;
// 	u8 TXState;

// 	TxData.StdId = ;
// 	TxData.ExtId = ;
// 	TxData.IDE = ;
// 	TxData.RTR = ;
// 	TxData.DLC = ;
// 	TxData.Data[7] = ;	
// 	TXState = CAN_Transmit(CAN1,&);

// 	while(CAN_TransmitStatus(CAN1,TXState) == CAN_TxStatus_Failed);
// }

// u8 CanRX()
// {
// 	CanRxMsg RxData;
// 	u8 TXState;

// 	memset((u8 *)&RxData,0,sizeof(CanRxMsg));
// 	if(CAN_MessagePending(CAN1,CAN_FIFO0) != 0)
// 	{
// 		CAN_Receive(CAN1,CAN_FIFO0,&RxData);
// 	}
// 	else
// 	{
// 		RxData.DLC = 0;
// 	}

// 	return RxData.DLC;
// }

// int fputc(int ch, FILE* stream)          
// {		
// 	u8 i=0;
// 	USART_ClearFlag(USART1,USART_FLAG_TC);
// 	USART_SendData(USART1, (unsigned char) ch);	
// 	while ((USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET) && ( i < 10))
// 	{
// 		// Delay_us(2);
// 	}
// 	USART_ClearFlag(USART1,USART_FLAG_TC);
//     return ch;
// }


// https://blog.csdn.net/Xuexi_touteng/article/details/136491667

//////////////////////////////////////////////
void Can_Init(u8 tsjw,u8 tbs1,u8 tbs2,u16 brp,u8 mode)
{
	// 结构体定义
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	// 开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);   // 打开CAN1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // PA端口时钟打开
	
	// 初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;   // PA11 CAN_RX   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;   // PA12 CAN_TX   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 初始化CAN
	CAN_InitStructure.CAN_TTCM=DISABLE;   // 非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;   // 软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;   // 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;   // 使用报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;   // 报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;   // 优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= CAN_Mode_LoopBack;   //CAN工作模式设置 
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;   // 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;   // Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;   // Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=4;   //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1
	
	// 初始化过滤器
	CAN_FilterInitStructure.CAN_FilterNumber=0;   // 过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;   // 掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;   // 32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;   // 32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;   // 过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;   // 激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);   // 过滤器初始化

	CAN_WakeUp(CAN1);
}

u8 Med_Can_Send_Msg (u8* msg,u8 len)
{	
	u8 mbox;
	u16 i = 0;
	CanTxMsg TxMessage;   // 定义发送报文结构体
	TxMessage.StdId = 0X1314;   // 标准标识符
	TxMessage.ExtId = 0X6666;   // 扩展标识符
	TxMessage.IDE = CAN_Id_Standard;   // 使用标准标识符
	TxMessage.RTR = CAN_RTR_Data;   // 消息类型为数据帧，一帧8位
	TxMessage.DLC = len;
	for(i = 0;i < len;i ++)
	{
		TxMessage.Data[i] = msg[i];   // 填充帧数据段
	}
	mbox = CAN_Transmit(CAN1,&TxMessage);   // 发送报文   
	i = 0;
	
	// 等待发送结束
	while((CAN_TransmitStatus(CAN1,mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
	{
		i++;
	}

	// 返回发送情况
	if(i >= 0XFFF)
	{
		return 1;
	}
	return 0;		
}

u8 Med_Can_Receive_Msg (u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;   // 定义接收报文结构体
	
	// 没有接收到数据,直接退出 
	if( CAN_MessagePending(CAN1,CAN_FIFO0) == 0)
	{
		return 0;
	}
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);   // 读取数据	
	
	for(i = 0;i < RxMessage.DLC;i ++)
	{
		buf[i] = RxMessage.Data[i];
	}
	
	return RxMessage.DLC;	
}
