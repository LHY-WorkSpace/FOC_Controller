#include "DataType.h"
#include "AS5600.h"
#include "gd32f30x.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Timer.h"


// SCL - PB11
// SDA - PB10
// 72Mhz = 210Khz
static void EC_IIC_Delay(u16 nus)
{
	u16 i,k;

	for(k=0; k<nus; k++)
	{
		for(i=0; i<2; i++)
		{
			__NOP();
		}
	}
}

static void EC_Start_IIC(void)
{
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SDA_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
}

static void EC_IIC_Stop(void)
{
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SDA_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
}

static void EC_IIC_Send_Ack(void)
{
	EC_IIC_SCL_LOW;
	EC_IIC_SDA_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
}

static void EC_IIC_Send_NAck(void)
{
	EC_IIC_SCL_LOW;
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
}

static u8 EC_IIC_Wait_Ack_OK(void)
{
	u8 i=0;

	EC_IIC_SCL_HIGH; 
	EC_IIC_Delay(2);                              
	while( EC_IIC_SDA_STATE == HIGH)
	{
			i++;
			if(i>10)
			{
				EC_IIC_Stop();
				return D_FALSE;
			}
			EC_IIC_Delay(2);	
	}			
	EC_IIC_SCL_LOW;	
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	return D_TRUE;
}

static void EC_IIC_SenddByte(u8 Data)
{
	u8 i=0;
	for(i=0;i<8;i++)
	{
		if(Data&0x80)	
		{
			EC_IIC_SDA_HIGH;
		}
		else
		{
			EC_IIC_SDA_LOW;
		}
		Data<<=1;
		EC_IIC_Delay(2);
		EC_IIC_SCL_HIGH;
		EC_IIC_Delay(2);
		EC_IIC_SCL_LOW;
	}
	EC_IIC_Delay(2);  
	EC_IIC_SDA_HIGH;               
}

static u8 EC_IIC_GetByte(void)
{
	u8 Data=0;
	u8 i=0;
	for(i=0;i<8;i++)
	{		
		Data<<=1;
		EC_IIC_SCL_LOW;
		EC_IIC_Delay(2); 		
		EC_IIC_SCL_HIGH;	
		EC_IIC_Delay(2);
		if( EC_IIC_SDA_STATE == HIGH)	
		{
			Data|=0x01;
		}
	}
 	EC_IIC_SCL_LOW;	
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
  return Data;
}

//************************// 
//  功能描述: AS5600_ IO 初始化函数
//  
//  参数: 无
//  
//  返回值: D_TRUE:成功  
//          D_FALSE:失败
//          0xFF:地址超范围
//			
//  说明: 
//
//************************//  
void AS5600_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA); 

	EC_IIC_SCL_HIGH;
	EC_IIC_SDA_HIGH;//置为总线空闲

    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
	Delay_ms(50);

	// 考虑后期是否加滤波
	// B16_B08 Config;
	// u8 Cmd = 0x04;
	// AS5600_WriteData(CONF_H_REG,1,&Cmd);
	// AS5600_ReadData(CONF_H_REG,2,&Config.B08[0]);
	// printf("Config:%x,%x\n",Config.B08[0],Config.B08[1]);
}

//************************// 
//  功能描述: AS5600_ 数据写入函数
//  
//  参数: 物理地址，长度，数据指针
//  
//  返回值: D_TRUE:成功  
//          D_FALSE:失败
//          0xFF:地址超范围
//			
//  说明: 
//
//************************//  
void AS5600_WriteData(u8 addr,u8 length,u8 *data)
{
	u8 i;
	EC_Start_IIC();
	EC_IIC_SenddByte( AS5600_ADDRESS << 1 );
    EC_IIC_Wait_Ack_OK();
	EC_IIC_SenddByte(addr);
	EC_IIC_Wait_Ack_OK();
	for(i=0;i<length;i++)
	{
		EC_IIC_SenddByte(data[i]);
		if(EC_IIC_Wait_Ack_OK() == D_FALSE)
		{
			return ;
		}
	}
	EC_IIC_Stop();
}



//************************// 
//  功能描述: AS5600_ 数据读取函数
//  
//  参数: 物理地址，长度，数据指针
//  
//  返回值: D_TRUE:成功  
//          D_FALSE:失败
//          0xFF:地址超范围
//			
//  说明: 
//
//************************//  
void AS5600_ReadData(u8 addr,u8 length,u8 *data)
{
	u8 i;

	EC_Start_IIC();
	EC_IIC_SenddByte( AS5600_ADDRESS << 1 );
    EC_IIC_Wait_Ack_OK();
	EC_IIC_SenddByte(addr);
	EC_IIC_Wait_Ack_OK();

	EC_Start_IIC();
	EC_IIC_SenddByte( (AS5600_ADDRESS << 1) | 0X01 );//读
    EC_IIC_Wait_Ack_OK();

	for(i=0;i<length;i++)
	{
		data[i]=EC_IIC_GetByte();
		if( i == length-1)
		{
			EC_IIC_Send_NAck();//最后一个字节发送N_ACK
		}
		else
		{
			EC_IIC_Send_Ack();
		}
	}
	EC_IIC_Stop();
}



//带圈数
// 500us
float AS5600_Angle(uint8_t Mode)
{
	static float TurnsNum =0.0;
	static float PrvAngle =0.0;	
	float Err;
	float AS5600Angle;
	float Result=0.0;
	B16_B08 AngleReg;

	memset(AngleReg.B08,0,sizeof(B16_B08));
	AS5600_ReadData(RAW_ANGLE_H_REG,2,&AngleReg.B08[0]);
	AngleReg.B16 = (AngleReg.B08[0]<<8)|AngleReg.B08[1];
	AS5600Angle = (float)AngleReg.B16*360.0f/4096.0f;

	Err = AS5600Angle - PrvAngle;

	if(fabs(Err) > 360.0*0.8)
	{
		TurnsNum += (Err > 0.0f) ? -1.0f:1.0f;
	}

	PrvAngle = AS5600Angle;

	switch (Mode)
	{
		case ANGLE_MODE:
			Result = AS5600Angle;
			break;
		case TURN_MODE:
			Result = TurnsNum;
			break;
		case ANGLE_TURN_MODE:
			Result = AS5600Angle + TurnsNum*360.0f;
			// printf("FOC:%.2f,%.2f\n",AS5600Angle,Result);
			break;
		default:
			break;
	}

	return Result;
}




void AS5600_Test()
{
	B16_B08 Angle;
	// u16 AngleTmp=0;
	
	memset(Angle.B08,0,sizeof(B16_B08));
	AS5600_ReadData(RAW_ANGLE_L_REG,1,&Angle.B08[0]);
	// Delay_ms(5);
	AS5600_ReadData(RAW_ANGLE_H_REG,1,&Angle.B08[1]);
	// Delay_ms(5);
	// AngleTmp = Angle.B16*360/4096;
	// printf("Angle_ADC:%d    Angle:%d\r\n",Angle.B16,AngleTmp);
	// Delay_ms(10);
}










