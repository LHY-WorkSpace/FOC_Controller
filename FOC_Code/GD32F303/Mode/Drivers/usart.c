#include "gd32f30x.h"
#include <stdio.h>
#include "DataType.h"
#include "usart.h"


// USART_Data_t USART1_Data;
// USART_Data_t USART2_Data;

void USART1_Init(u32 bode)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, bode);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}



#if 0


/*
	USART2_MODE_A : PD5-TX2 
	         		PD6-RX2

	USART2_MODE_B : PA2-TX2 
	        		PA3-RX2	
*/
void USART2_Init(u32 bode,u16 DataLength,u16 StopBit,u16 Parity)
{
		GPIO_InitTypeDef USART_GPIO_Init;
		USART_InitTypeDef USART2_Initstruc;
		NVIC_InitTypeDef  NVIC_Initstr;

	if(DataLength==USART_DATA_8bit )
	{
		if(Parity!=USART_PARTYT_NO)
		{
			Parity=USART_PARTYT_NO;
			//return;                   //8λ���ݱ�����У�飬9λ��������У��
		}
			
	}

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	#ifdef USART2_MODE_A
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
		USART_GPIO_Init.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6;
	#elif defined USART2_MODE_B
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		USART_GPIO_Init.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	#endif

		USART_GPIO_Init.GPIO_Mode=GPIO_Mode_AF;                           
		USART_GPIO_Init.GPIO_PuPd=GPIO_PuPd_UP;
		USART_GPIO_Init.GPIO_Speed=GPIO_Speed_50MHz;

	#ifdef USART2_MODE_A
		GPIO_Init(GPIOD,&USART_GPIO_Init);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
	#elif defined USART2_MODE_B
		GPIO_Init(GPIOA,&USART_GPIO_Init);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	#endif	
	

	USART2_Initstruc.USART_BaudRate=bode;                                  
	USART2_Initstruc.USART_WordLength=DataLength;
	USART2_Initstruc.USART_StopBits=StopBit;
	USART2_Initstruc.USART_Parity=Parity;
	USART2_Initstruc.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART2_Initstruc.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART2,&USART2_Initstruc);
	
	NVIC_Initstr.NVIC_IRQChannel=USART2_IRQn;
	NVIC_Initstr.NVIC_IRQChannelPreemptionPriority=6;
	NVIC_Initstr.NVIC_IRQChannelSubPriority=0;
	NVIC_Initstr.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Initstr);

	USART_ClearFlag(USART2,0x3ff);
	USART_ITConfig(USART2,USART_IT_RXNE ,ENABLE);
	USART_ITConfig(USART2,USART_IT_IDLE, ENABLE);//��֧�ֻ� | ������
	USART_Cmd(USART2,ENABLE);	
}


//************************// 
//  ��������: USARTͨ�ô�������
//  
//  ����: ���ں� ������Ϣ�ṹ��
//  
//  ����ֵ: ��
//  
//  ˵��: ���ڿ��ǵ��γ����Ĵ����Լ�DMA����
// 
//************************//  

void USARTx_ITHandle(USART_TypeDef* USARTx,USART_Data_t *USART_Data)
{
	u16 i;

	//�����ж�
	if(USART_GetITStatus(USARTx,USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USARTx,USART_IT_RXNE);
		USART_Data->RX_Data[ USART_Data->RX_Pointer++ ] = USART_ReceiveData(USARTx);
		// USART_Data->RX_Pointer %= BUFFER_SIZE;//���ڿ��ǵ������ݳ�����������
	}

	//��������ж�
	if(USART_GetITStatus(USARTx,USART_IT_TXE) != RESET)
	{
		USART_ClearITPendingBit(USARTx,USART_IT_TXE);

		USART_SendData(USARTx,USART_Data->TX_Data[ USART_Data->TX_Pointer++ ]);

		if( USART_Data->TX_Pointer >= USART_Data->TX_Length)
		{
			USART_Data->TX_Length = USART_Data->TX_Pointer;
			USART_Data->TX_Pointer = 0;
			USART_ITConfig(USARTx,USART_IT_TXE,DISABLE);
		}
	}

	//���տ����ж�
	if(USART_GetITStatus(USARTx,USART_IT_IDLE) != RESET)
	{
		//�˴������ȶ�SR�ٶ�DR�����־λ
		i = USARTx->SR;
		i = USARTx->DR;
		i++;
		//�˴�����ʹ��DMA
		USART_Data->RX_Length = USART_Data->RX_Pointer;
		USART_Data->RX_Pointer = 0;
	}
}

//************************// 
//  ��������: �����жϷ��ͺ���
//  
//  ����: ���ں�,������Ϣ�ṹ��ָ��,��������ָ��,���ݳ���
//  
//  ����ֵ: TRUE:�ɹ�
//			OVER_FLOW:���ݳ���
//			BUSY:������æ
//  ˵��: 
// 
//************************//  
u8 USART_ITSendData(USART_TypeDef* USARTx,USART_Data_t *USART_Data,u16 Length,u8 *Data)
{
	if(Length > BUFFER_SIZE)
	{
		return OVER_FLOW;
	}

	if( USART_Data->TX_Pointer != 0)//���ڷ�������
	{
		return BUSY;
	}

	memcpy(USART_Data->TX_Data,Data,Length);
	USART_Data->TX_Pointer = 0;
	USART_Data->TX_Length = Length;
	USART_ITConfig(USARTx,USART_IT_TXE,ENABLE);
	return TRUE;
}

//************************// 
//  ��������: ���жϴ��ڷ��ͺ���
//  
//  ����: ���ں�,������Ϣ�ṹ��ָ��,��������ָ��,���ݳ���
//  
//  ����ֵ: ��
//  
//  ˵��: ��
//
//************************//  
void USART_PollingSendData(USART_TypeDef* USARTx,USART_Data_t *USART_Data,u8 *Data,u16 Length)
{
	u16 i,k=0;
	for ( i = 0; i < Length; i++)
	{
		USART_ClearFlag(USARTx,USART_FLAG_TC);
		USART_SendData(USARTx, *(Data+i));
		while ((USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET) && ( k < 10))
		{
			Delay_ms(2);
			k++;
		}
	}
}

//***************************************************//
//  ��������: ��ȡ����״̬
//  
//  ����: 
//  
//  ����ֵ: TRUE / FALSE
//  
//  ˵��: ֻ�н�����ɺ�����к�����ȡ����
//  
//***************************************************//
u8 USART_RxCompleteFlag(USART_Data_t *USART_Data)
{
	if( USART_Data->RX_Length  == 0 )
	{
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}

//***************************************************//
//  ��������: ��ȡ�������ݻ���ĵ�ַ
//  
//  ����: offset������ƫ��
//  
//  ����ֵ: RX_Dataƫ�ƶ�Ӧ�����ݵ�ַ
//  
//  ˵��: ��
//  
//***************************************************//
u8 *USART_RxDataAddr(USART_Data_t *USART_Data,u16 offset)
{
	if(offset >= BUFFER_SIZE )
	{
		return (USART_Data->RX_Data);
	}
	else
	{
		return (USART_Data->RX_Data + offset);
	}
}
//***************************************************//
//  ��������: ��ȡ�������ݻ���ĵ�ַ
//  
//  ����: offset������ƫ��
//  
//  ����ֵ: TX_Dataƫ�ƶ�Ӧ�����ݵ�ַ
//  
//  ˵��: ��
//  
//***************************************************//
u8 *USART_TxDataAddr(USART_Data_t *USART_Data,u16 offset)
{
	if(offset >= BUFFER_SIZE )
	{
		return (USART_Data->TX_Data);
	}
	else
	{
		return (USART_Data->TX_Data + offset);
	}
}

//************************// 
//  ��������: ���ڽ��պ���
//  
//  ����: ������Ϣ�ṹ��ָ��,��������С�����ݵ�ַ��ʵ�ʽ��ճ���
//  
//  ����ֵ: TRUE:�ɹ�
//			BUSY:������æ
//			ILDE:����
//
//  ˵��: 
//
//************************//  
u8 USART_GetData(USART_Data_t *USART_Data,u16 Buffsize,u8 *Data,u16 *Length)
{

	if( Buffsize > BUFFER_SIZE)
	{
		memset(Data,0x00,Buffsize);
		*Length = 0;
		return FALSE;
	}
	
	memset(Data,0x00,Buffsize);
	
	if( USART_Data->RX_Length  == 0 )
	{
		*Length = 0;

		if( USART_Data->RX_Pointer != 0)
		{
			return BUSY;//���ڽ�������
		}
		else
		{
			return ILDE;//����(�ϴ�ȡ�����ݺ�û�н��յ�������)
		}
	}
	else
	{
		if( Buffsize >= USART_Data->RX_Length )//ʵ�ʽ������ݳ���С�ڻ��峤�ȣ��򷵻�ʵ�ʳ���
		{
			*Length = USART_Data->RX_Length;
			memcpy(Data,USART_Data->RX_Data,USART_Data->RX_Length);
			USART_Data->RX_Length = 0;
			return TRUE;
		}
		else//ʵ�ʽ������ݳ��ȴ��ڻ��峤�ȣ��򷵻ػ��峤��
		{
			*Length = Buffsize;
			memcpy(Data,USART_Data->RX_Data,Buffsize);
			USART_Data->RX_Length = 0;
			return OVER_FLOW;
		}
	}

}
//************************// 
//  ��������: USARTx_IRQ ����
//  
//  ����: ��
//  
//  ����ֵ: ��
//  
//  ˵��: ��
//
//************************//  
void USART1_IRQHandler()
{
	USARTx_ITHandle(USART1,&USART1_Data);
}

// void USART2_IRQHandler()
// {
// 	USARTx_ITHandle(USART2,&USART2_Data);
// }



#endif