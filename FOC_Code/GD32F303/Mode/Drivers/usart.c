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
	usart_parity_config(USART0, USART_PM_NONE);
	usart_word_length_set(USART0, USART_WL_8BIT);
	usart_stop_bit_set(USART0, USART_STB_1BIT);

    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

	nvic_irq_enable(USART0_IRQn, 0, 0);
	
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
			//return;                   //8位数据必须无校验，9位数据任意校验
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
	USART_ITConfig(USART2,USART_IT_IDLE, ENABLE);//不支持或（ | ）操作
	USART_Cmd(USART2,ENABLE);	
}


//************************// 
//  功能描述: USART通用处理函数
//  
//  参数: 串口号 串口信息结构体
//  
//  返回值: 无
//  
//  说明: 后期考虑单次超长的处理以及DMA搬运
// 
//************************//  

void USARTx_ITHandle(USART_TypeDef* USARTx,USART_Data_t *USART_Data)
{
	u16 i;

	//接收中断
	if(USART_GetITStatus(USARTx,USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USARTx,USART_IT_RXNE);
		USART_Data->RX_Data[ USART_Data->RX_Pointer++ ] = USART_ReceiveData(USARTx);
		// USART_Data->RX_Pointer %= BUFFER_SIZE;//后期考虑单次数据长度溢出的情况
	}

	//发送完成中断
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

	//接收空闲中断
	if(USART_GetITStatus(USARTx,USART_IT_IDLE) != RESET)
	{
		//此处必须先读SR再度DR来清标志位
		i = USARTx->SR;
		i = USARTx->DR;
		i++;
		//此处可以使用DMA
		USART_Data->RX_Length = USART_Data->RX_Pointer;
		USART_Data->RX_Pointer = 0;
	}
}

//************************// 
//  功能描述: 启动中断发送函数
//  
//  参数: 串口号,串口信息结构体指针,发送数据指针,数据长度
//  
//  返回值: TRUE:成功
//			OVER_FLOW:数据超长
//			BUSY:发送正忙
//  说明: 
// 
//************************//  
u8 USART_ITSendData(USART_TypeDef* USARTx,USART_Data_t *USART_Data,u16 Length,u8 *Data)
{
	if(Length > BUFFER_SIZE)
	{
		return OVER_FLOW;
	}

	if( USART_Data->TX_Pointer != 0)//正在发送数据
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
//  功能描述: 非中断串口发送函数
//  
//  参数: 串口号,串口信息结构体指针,发送数据指针,数据长度
//  
//  返回值: 无
//  
//  说明: 无
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
//  功能描述: 获取接受状态
//  
//  参数: 
//  
//  返回值: TRUE / FALSE
//  
//  说明: 只有接收完成后才能有后续读取操作
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
//  功能描述: 获取接收数据缓冲的地址
//  
//  参数: offset：缓冲偏移
//  
//  返回值: RX_Data偏移对应的数据地址
//  
//  说明: 无
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
//  功能描述: 获取发送数据缓冲的地址
//  
//  参数: offset：缓冲偏移
//  
//  返回值: TX_Data偏移对应的数据地址
//  
//  说明: 无
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
//  功能描述: 串口接收函数
//  
//  参数: 串口信息结构体指针,缓冲区大小，数据地址，实际接收长度
//  
//  返回值: TRUE:成功
//			BUSY:接收正忙
//			ILDE:空闲
//
//  说明: 
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
			return BUSY;//正在接收数据
		}
		else
		{
			return ILDE;//空闲(上次取走数据后，没有接收到新数据)
		}
	}
	else
	{
		if( Buffsize >= USART_Data->RX_Length )//实际接收数据长度小于缓冲长度，则返回实际长度
		{
			*Length = USART_Data->RX_Length;
			memcpy(Data,USART_Data->RX_Data,USART_Data->RX_Length);
			USART_Data->RX_Length = 0;
			return TRUE;
		}
		else//实际接收数据长度大于缓冲长度，则返回缓冲长度
		{
			*Length = Buffsize;
			memcpy(Data,USART_Data->RX_Data,Buffsize);
			USART_Data->RX_Length = 0;
			return OVER_FLOW;
		}
	}

}
//************************// 
//  功能描述: USARTx_IRQ 函数
//  
//  参数: 无
//  
//  返回值: 无
//  
//  说明: 无
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
