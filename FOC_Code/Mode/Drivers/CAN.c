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
	// �ṹ�嶨��
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	// ����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);   // ��CAN1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // PA�˿�ʱ�Ӵ�
	
	// ��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;   // PA11 CAN_RX   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // ��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;   // PA12 CAN_TX   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   // �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// ��ʼ��CAN
	CAN_InitStructure.CAN_TTCM=DISABLE;   // ��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;   // ����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE;   // ˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;   // ʹ�ñ����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;   // ���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;   // ���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_LoopBack;   //CAN����ģʽ���� 
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;   // ����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;   // Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;   // Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=4;   //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1
	
	// ��ʼ��������
	CAN_FilterInitStructure.CAN_FilterNumber=0;   // ������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;   // ����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;   // 32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;   // 32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;   // ������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;   // ���������0
	CAN_FilterInit(&CAN_FilterInitStructure);   // ��������ʼ��

	CAN_WakeUp(CAN1);
}

u8 Med_Can_Send_Msg (u8* msg,u8 len)
{	
	u8 mbox;
	u16 i = 0;
	CanTxMsg TxMessage;   // ���巢�ͱ��Ľṹ��
	TxMessage.StdId = 0X1314;   // ��׼��ʶ��
	TxMessage.ExtId = 0X6666;   // ��չ��ʶ��
	TxMessage.IDE = CAN_Id_Standard;   // ʹ�ñ�׼��ʶ��
	TxMessage.RTR = CAN_RTR_Data;   // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC = len;
	for(i = 0;i < len;i ++)
	{
		TxMessage.Data[i] = msg[i];   // ���֡���ݶ�
	}
	mbox = CAN_Transmit(CAN1,&TxMessage);   // ���ͱ���   
	i = 0;
	
	// �ȴ����ͽ���
	while((CAN_TransmitStatus(CAN1,mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
	{
		i++;
	}

	// ���ط������
	if(i >= 0XFFF)
	{
		return 1;
	}
	return 0;		
}

u8 Med_Can_Receive_Msg (u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;   // ������ձ��Ľṹ��
	
	// û�н��յ�����,ֱ���˳� 
	if( CAN_MessagePending(CAN1,CAN_FIFO0) == 0)
	{
		return 0;
	}
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);   // ��ȡ����	
	
	for(i = 0;i < RxMessage.DLC;i ++)
	{
		buf[i] = RxMessage.Data[i];
	}
	
	return RxMessage.DLC;	
}
