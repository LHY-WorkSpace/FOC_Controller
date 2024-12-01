#include "DataType.h"
#include "eeprom.h"
#include "gd32f30x.h"

// SCL - PB0
// SDA - PB1
//72Mhz = 133khz
static void EE_IIC_Delay(u16 nus)
{
	u16 i,k;

	for(k=0; k<nus; k++)
	{
		for(i=0; i<10; i++)
		{
			__NOP();
		}
	}
}

static void EE_Start_IIC(void)
{
	EE_IIC_SDA_HIGH;
	EE_IIC_Delay(2);
	EE_IIC_SCL_HIGH;
	EE_IIC_Delay(2);
	EE_IIC_SDA_LOW;
	EE_IIC_Delay(2);
	EE_IIC_SCL_LOW;
	EE_IIC_Delay(2);

}

static void EE_IIC_Stop(void)
{
	EE_IIC_SCL_LOW;
	EE_IIC_Delay(2);
	EE_IIC_SDA_LOW;
	EE_IIC_Delay(2);
	EE_IIC_SCL_HIGH;
	EE_IIC_Delay(2);
	EE_IIC_SDA_HIGH;
	EE_IIC_Delay(2);

}

static void EE_IIC_Send_Ack(void)
{
	EE_IIC_SCL_LOW;
	EE_IIC_SDA_LOW;
	EE_IIC_Delay(2);
	EE_IIC_SCL_HIGH;
	EE_IIC_Delay(2);
	EE_IIC_SCL_LOW;
	EE_IIC_Delay(2);
	EE_IIC_SDA_HIGH;
	EE_IIC_Delay(2);
}

static void EE_IIC_Send_NAck(void)
{
	EE_IIC_SCL_LOW;
	EE_IIC_SDA_HIGH;
	EE_IIC_Delay(2);
	EE_IIC_SCL_HIGH;
	EE_IIC_Delay(2);
	EE_IIC_SCL_LOW;
	EE_IIC_Delay(2);
	EE_IIC_SDA_HIGH;
	EE_IIC_Delay(2);
}

static u8 EE_IIC_Wait_Ack_OK(void)
{
	u8 i=0;

	EE_IIC_SCL_HIGH; 
	EE_IIC_Delay(2);                              
	while( EE_IIC_SDA_STATE == HIGH)
	{
			i++;
			if(i>10)
			{
				EE_IIC_Stop();
				return D_FALSE;
			}
			EE_IIC_Delay(2);	
	}			
	EE_IIC_SCL_LOW;	
	EE_IIC_Delay(2);
	EE_IIC_SDA_HIGH;
	return D_TRUE;
}

static void EE_IIC_SenddByte(u8 Data)
{

	u8 i=0;
	
	for(i=0;i<8;i++)
	{
		//EE_IIC_Delay(2);
		if(Data&0x80)	
		{
			EE_IIC_SDA_HIGH;
		}
		else
		{
			EE_IIC_SDA_LOW;
		}
		Data<<=1;
		EE_IIC_Delay(2);
		EE_IIC_SCL_HIGH;
		EE_IIC_Delay(2);
		EE_IIC_SCL_LOW;
	}
	EE_IIC_Delay(2);  
	EE_IIC_SDA_HIGH;               
}


static u8 EE_IIC_GetByte(void)
{
	u8 Data=0;
	u8 i=0;
	for(i=0;i<8;i++)
	{		
		Data<<=1;
		EE_IIC_SCL_LOW;
		EE_IIC_Delay(2); 		
		EE_IIC_SCL_HIGH;	
		EE_IIC_Delay(2);
		if( EE_IIC_SDA_STATE == HIGH)	
		{
			Data|=0x01;
		}
	}
 	EE_IIC_SCL_LOW;	
	EE_IIC_Delay(2);
	EE_IIC_SDA_HIGH;
  return Data;
}



void EEPROM_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA); 

	EE_IIC_SCL_HIGH;
	EE_IIC_SDA_HIGH;//��Ϊ���߿���

    gpio_init(GPIOA, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
	gpio_init(GPIOA, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
}

//************************// 
//  ��������: EEPROM ����д�뺯��
//  
//  ����: �����ַ�����ȣ�����ָ��
//  
//  ����ֵ: 0:�ɹ�  
//          1:ʧ��
//          0xFF:��ַ����Χ
//			
//  ˵��: 
//
//************************//  
u8  EE_WriteData(u16 addr,u16 length,u8 *Data)
{
	u8 k,i;
	u16 PageNum,WR_Len,Offset,LenCount;
	B16_B08 MemAddr;	//���Ե�ַ

	if( addr >= EEPROM_PAGE_SIZE*EEPROM_PAGES)
	{
		return 0XFF;
	}
	
	MemAddr.B16 = addr;

	PageNum = addr/EEPROM_PAGE_SIZE;			//�õ�ҳ���
	Offset = addr - PageNum*EEPROM_PAGE_SIZE;	//����ҳ��ƫ��
	WR_Len = EEPROM_PAGE_SIZE - Offset;		//����һҳ�ڴ�ƫ��λ��Ҫд����ֽ���
	LenCount = 0;

	if( WR_Len >= length )	//����С�ڵ���δ��ҳʣ���ֽ�������ʵ�ʳ���д��
	{
		WR_Len = length;
	}

	do
	{
		EE_Start_IIC();
		EE_IIC_SenddByte(EEPROM_ADDRESS );
		EE_IIC_Wait_Ack_OK();
		EE_IIC_SenddByte(MemAddr.B08[1]);
		EE_IIC_Wait_Ack_OK();
		EE_IIC_SenddByte(MemAddr.B08[0]);
		EE_IIC_Wait_Ack_OK();

		for(k=0;k<WR_Len;k++)
		{
			EE_IIC_SenddByte(Data[LenCount]);
			if(EE_IIC_Wait_Ack_OK() == D_FALSE)
			{
				return D_FALSE;
			}
			LenCount++;
		}
		EE_IIC_Stop();
		//ҳд��ʱ�������� 5 ms;
		for ( i = 0; i < 5; i++)
		{
			EE_IIC_Delay(1000);
		}
		
		MemAddr.B16 += WR_Len;

		if( (length - LenCount) >= EEPROM_PAGE_SIZE)
		{
			WR_Len = EEPROM_PAGE_SIZE;
		}
		else
		{
			WR_Len = length - LenCount;
		}

	}while(LenCount != length);

	return D_TRUE;
}



//************************// 
//  ��������: EEPROM ���ݶ�ȡ����
//  
//  ����: �����ַ�����ȣ�����ָ��
//  
//  ����ֵ: 0:�ɹ�  
//          1:ʧ��
//          0xFF:��ַ����Χ
//			
//  ˵��: 
//
//************************//  
u8 EE_ReadData(u16 addr,u16 length,u8 *Data)
{
	u16 i;
	B16_B08 MemAddr;	//���Ե�ַ

	if( addr >= EEPROM_PAGE_SIZE*EEPROM_PAGES)
	{
		return 0XFF;
	}

	MemAddr.B16=addr;

	EE_Start_IIC();
	EE_IIC_SenddByte(EEPROM_ADDRESS );
    EE_IIC_Wait_Ack_OK();
	EE_IIC_SenddByte(MemAddr.B08[1]);
    EE_IIC_Wait_Ack_OK();
	EE_IIC_SenddByte(MemAddr.B08[0]);
	EE_IIC_Wait_Ack_OK();

	EE_Start_IIC();
	EE_IIC_SenddByte(EEPROM_ADDRESS|0X01);//��ȡ����
    EE_IIC_Wait_Ack_OK();

	for(i=0;i<length;i++)
	{
		Data[i]=EE_IIC_GetByte();
		if( i == length-1)
		{
			EE_IIC_Send_NAck();//���һ���ֽڷ���N_ACK
		}
		else
		{
			EE_IIC_Send_Ack();
		}
	}
	EE_IIC_Stop();
	return D_TRUE;
}












