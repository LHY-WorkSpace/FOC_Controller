#ifndef  AS5600_H
#define  AS5600_H


#define AS5600_ADDRESS                   (0x36)  //设备地址


//========= Configuration Registers =========
#define ZMCO_REG        (0x00)//

#define ZPOS_H_REG      (0x01)//
#define ZPOS_L_REG      (0x02)//

#define MPOS_H_REG      (0x03)//
#define MPOS_L_REG      (0x04)//

#define NABG_H_REG      (0x05)//
#define MANG_L_REG      (0x06)//

#define CONF_H_REG      (0x07)//
#define CONF_L_REG      (0x08)//



//========= Data Registers =========
#define RAW_ANGLE_H_REG      (0x0C)//
#define RAW_ANGLE_L_REG      (0x0D)//

#define ANGLE_H_REG      (0x0E)//
#define ANGLE_L_REG      (0x0F)//


//========= Status Registers =========
#define STATUS_REG      (0x0B)//

#define AGC_REG         (0x1A)//

#define MAGNITUDE_H_REG      (0x1B)//
#define MAGNITUDE_L_REG      (0x1C)//

//========= Burn Commands =========
#define BURN            (0xFF)//


// SCL - PB11
// SDA - PB10

#define EC_IIC_SCL_LOW  			gpio_bit_reset(GPIOB,GPIO_PIN_1)
#define EC_IIC_SCL_HIGH  		 	gpio_bit_set(GPIOB,GPIO_PIN_1)
#define EC_IIC_SDA_LOW  			gpio_bit_reset(GPIOB,GPIO_PIN_1)
#define EC_IIC_SDA_HIGH 			gpio_bit_set(GPIOB,GPIO_PIN_1)
#define EC_IIC_SDA_STATE            gpio_input_bit_get(GPIOB,GPIO_PIN_1)

#define ANGLE_MODE          (0x01)//角度
#define TURN_MODE           (0x02)//圈数
#define ANGLE_TURN_MODE     (0x03)//带圈数的角度

void AS5600_Init(void);
void  AS5600_WriteData(u8 addr,u8 length,u8 *data);
void AS5600_ReadData(u8 addr,u8 length,u8 *data);
float AS5600_Angle(u8 Mode);
void AS5600_Test(void);
#endif



