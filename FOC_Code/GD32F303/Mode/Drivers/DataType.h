#ifndef  DATATYPE_H
#define  DATATYPE_H


#define  D_TRUE       0
#define  D_FALSE      1

#define  HIGH   1
#define  LOW    0

#define	Bit_0	0x0001
#define	Bit_1	0x0002
#define	Bit_2	0x0004
#define	Bit_3	0x0008
#define	Bit_4	0x0010
#define	Bit_5	0x0020
#define	Bit_6	0x0040
#define	Bit_7	0x0080
#define	Bit_8	0x0100
#define	Bit_9	0x0200
#define	Bit_10	0x0400
#define	Bit_11	0x0800
#define	Bit_12	0x1000
#define	Bit_13	0x2000
#define	Bit_14	0x4000
#define	Bit_15	0x8000

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

// typedef char s8;
// typedef short s16;
// typedef int s32;


typedef union 
{
    u8  B08[2];
    u16 B16;
}B16_B08;

typedef union 
{
    u8  B08[4];
    u32 B32;
}B32_B08;

typedef union 
{
    u16 B16[2];
    u32 B32;
}B32_B16;

#define MODE_1      (0xAA)
#define MODE_2      (0x55)//速度快，2位小数精度
#define MODE_FPU    (0x11)

// 三角函数计算方式选择
#define CALCULATE_MODE     (MODE_FPU)

//弧度->角度
#define RADTODEG(x) ((x) * 57.295779513082320876798154814105f)
//角度->弧度
#define DEGTORAD(x) ((x) * 0.01745329251994329576923690768489f)

#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

float FastSin(float x);
float FastCos(float x);
void FastSinCos(float InVal, float* SinP, float* CosP);








#define  CLOSE_ALL_IRQ      do{  __disable_irq();  }while(0)    
#define  OPEN_ALL_IRQ       do{  __enable_irq();  }while(0)    



#endif






