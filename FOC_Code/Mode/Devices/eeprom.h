#ifndef EEPROM_H
#define EEPROM_H

// SCL - PB0
// SDA - PB1

#define EEPROM_SIZE (32 * 1024)                         // 字节
#define EEPROM_PAGE_SIZE (64)                           // 字节
#define EEPROM_PAGES (EEPROM_SIZE / EEPROM_PAGE_SIZE) // 页数 内部Ram按页(EEPROM_PAGE_SIZE Byte)对齐
#define EEPROM_ADDRESS (0xA0)                           // AT24C02 设备地址


#define EE_IIC_SCL_LOW  			GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define EE_IIC_SCL_HIGH  		 	GPIO_SetBits(GPIOB,GPIO_Pin_0)
#define EE_IIC_SDA_LOW  			GPIO_ResetBits(GPIOB,GPIO_Pin_1)
#define EE_IIC_SDA_HIGH 			GPIO_SetBits(GPIOB,GPIO_Pin_1)
#define EE_IIC_SDA_STATE            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)

// #define EEPROM_ADDR(NUMBER) ((u16)(((uint32_t)&(((EEPROM_MAP *)0)->NUMBER))&0x0000FFFF))
#pragma pack(1)
typedef union
{
    u8 EEPROM_MEM[1024];
}EEPROM_MAP;

#pragma pack()

#define EEPROM_ADDR(NUMBER) ((uint16_t) & (((EEPROM_MAP *)0)->NUMBER))



void EEPROM_Init(void);
u8 EE_WriteData(u16 addr, u16 length, u8 *data);
u8 EE_ReadData(u16 addr, u16 length, u8 *data);

#endif
