#include "u8g2_UserGUI.h"
#include "DataType.h"
#include "stm32f10x.h"
#include "Timer.h"
#include "transform_3D.h"
#include "ADC.h"
static u8g2_t u8g2_Data;


u8 flag=0;
static u8 FreeRTOS_Logo[] =
{
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XFF,0XFF,0XFF,0X05,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0X00,0X00,0X00,0X00,
0X00,0X20,0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X01,
0X00,0X10,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XE0,0XFF,0XFF,0XFF,0X07,
0X00,0X18,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X18,
0X00,0X18,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X38,
0X00,0X0C,0XC0,0X9F,0XFF,0X07,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X38,
0X00,0X0C,0XE0,0X8D,0XFF,0X3F,0XF8,0XFF,0X1F,0XC0,0X1F,0X00,0X00,0X00,0X00,0X38,
0X00,0X0C,0XB0,0XCD,0XFF,0X7F,0XF8,0XFF,0X1F,0XF8,0XFF,0X01,0XC0,0XFF,0X01,0X78,
0X00,0X06,0XB0,0XCF,0XFF,0XFF,0XFC,0XFF,0X1F,0XFE,0XFF,0X07,0XF8,0XFF,0X07,0X78,
0X00,0X06,0XF0,0XE7,0X0F,0XFF,0XF8,0XFF,0X0F,0XFF,0XFF,0X0F,0XFC,0XFF,0X07,0X78,
0X00,0X06,0XE0,0XE3,0X07,0X7E,0X00,0X3F,0X80,0XFF,0XF8,0X0F,0XFE,0XFF,0X03,0X38,
0X00,0X03,0X00,0XE0,0X07,0X7E,0X80,0X3F,0XC0,0X3F,0XE0,0X1F,0XFF,0X81,0X03,0X38,
0X00,0X03,0X70,0XF0,0X07,0X3F,0X80,0X3F,0XE0,0X1F,0XC0,0X1F,0XFF,0X00,0X00,0X38,
0X80,0X01,0X7C,0XF3,0XFF,0X1F,0X80,0X1F,0XE0,0X0F,0XC0,0X1F,0XFF,0X00,0X00,0X38,
0X80,0X01,0X2E,0XF3,0XFF,0X0F,0X80,0X1F,0XF0,0X07,0XC0,0X1F,0XFF,0X03,0X00,0X38,
0X80,0X01,0XB6,0XF9,0XFF,0X07,0XC0,0X1F,0XF0,0X07,0XC0,0X1F,0XFF,0X1F,0X00,0X38,
0XC0,0X00,0XFE,0XF9,0XFF,0X07,0XC0,0X1F,0XF0,0X07,0XC0,0X1F,0XFE,0XFF,0X00,0X3C,
0XC0,0X00,0XFE,0XF8,0XF1,0X07,0XC0,0X0F,0XF0,0X07,0XC0,0X1F,0XFC,0XFF,0X01,0X3C,
0XE0,0X00,0X3D,0XFC,0XE1,0X07,0XE0,0X0F,0XF0,0X07,0XE0,0X1F,0XF0,0XFF,0X03,0X3C,
0X60,0X00,0X03,0XFC,0XE0,0X0F,0XE0,0X0F,0XF0,0X07,0XE0,0X1F,0X00,0XFF,0X07,0X3C,
0X60,0X00,0X03,0XFC,0XE0,0X0F,0XE0,0X0F,0XF0,0X07,0XF0,0X0F,0X00,0XFC,0X07,0X3C,
0X70,0X80,0X7F,0XFE,0XE0,0X0F,0XE0,0X07,0XF0,0X0F,0XF8,0X0F,0X00,0XF8,0X07,0X3C,
0X30,0X80,0X7F,0X7E,0XE0,0X0F,0XF0,0X07,0XF0,0X1F,0XFE,0X87,0X00,0XF8,0X07,0X3C,
0X30,0X18,0X3F,0X7E,0XE0,0X0F,0XF0,0X07,0XE0,0XFF,0XFF,0X83,0X07,0XFC,0X07,0X3C,
0X18,0X1C,0X00,0X00,0XC0,0X0F,0XF0,0X07,0XC0,0XFF,0XFF,0X81,0XFF,0XFF,0X03,0X1C,
0X18,0XCC,0X00,0X00,0X00,0X00,0XE0,0X03,0X80,0XFF,0X7F,0XC0,0XFF,0XFF,0X01,0X1C,
0X1C,0XFC,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0XFC,0X1F,0XC0,0XFF,0XFF,0X00,0X1C,
0X0C,0XFC,0X9F,0XFF,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X3F,0X00,0X1E,
0X0C,0XF8,0X9F,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X80,0X01,0X00,0X1E,
0X0E,0X30,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,0X00,0X00,0X00,0X00,0X00,0X1E,
0X06,0X00,0X00,0X00,0X00,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,0X00,0X00,0X00,0X1E,
0X06,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0X0F,0X00,0X1E,
0X06,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0XFF,0XFF,0X03,0X1E,
0X06,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0X03,0X1E,
0XFE,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0X03,0X1E,
0XFC,0XFF,0XFF,0X3F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1E,
0XFC,0XFF,0XFF,0XFF,0XFF,0X3F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0E,
0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0E,
0X00,0X00,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X0F,0X00,0X00,0X00,0X00,0X00,0X0E,
0X00,0X00,0X00,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X07,0X00,0X00,0X00,0X0F,
0X00,0X00,0X00,0X00,0X00,0XF0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,0X00,0X0F,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X07,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF,0X01,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF0,0XFF,0XFF,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};

const u8 NumCode[][16]=
{
    {0x00,0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18,0x00,0x00},/*"0",0*/

    {0x00,0x00,0x00,0x08,0x0E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00},/*"1",1*/

    {0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x20,0x20,0x10,0x08,0x04,0x42,0x7E,0x00,0x00},/*"2",2*/

    {0x00,0x00,0x00,0x3C,0x42,0x42,0x20,0x18,0x20,0x40,0x40,0x42,0x22,0x1C,0x00,0x00},/*"3",3*/

    {0x00,0x00,0x00,0x20,0x30,0x28,0x24,0x24,0x22,0x22,0x7E,0x20,0x20,0x78,0x00,0x00},/*"4",4*/

    {0x00,0x00,0x00,0x7E,0x02,0x02,0x02,0x1A,0x26,0x40,0x40,0x42,0x22,0x1C,0x00,0x00},/*"5",5*/

    {0x00,0x00,0x00,0x38,0x24,0x02,0x02,0x1A,0x26,0x42,0x42,0x42,0x24,0x18,0x00,0x00},/*"6",6*/

    {0x00,0x00,0x00,0x00,0x7E,0x22,0x22,0x10,0x10,0x08,0x08,0x08,0x08,0x08,0x08,0x00},/*"7",7*/

    {0x00,0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x24,0x18,0x24,0x42,0x42,0x42,0x3C,0x00},/*"8",8*/

    {0x00,0x00,0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x64,0x58,0x40,0x40,0x24,0x1C,0x00},/*"9",9*/
};


void Power_On(void);

GUI_t GUI_Info =
{   
    .KeyInfo.KeyNum = Key_MaxNum,
    .KeyInfo.KeyState = NONE_PRESS,
    .Index = Main_ui,
    .UI_List[Main_ui] = Power_On,
    // .UI_List[Main_ui] = Main_UI,

};

// PA5-SPI_CLK 
// PA4-SPI_RST 
// PA6-SPI_DC 
// PA7-SPI_MOSI

void OLED_Init()
{
	GPIO_InitTypeDef SPI_GPIO_Init;
    SPI_InitTypeDef SPI_InitDef;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	SPI_GPIO_Init.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
	SPI_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	SPI_GPIO_Init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&SPI_GPIO_Init);

	SPI_GPIO_Init.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_6;
	SPI_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	SPI_GPIO_Init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&SPI_GPIO_Init);

    SPI_InitDef.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitDef.SPI_Mode = SPI_Mode_Master;
    SPI_InitDef.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitDef.SPI_CPOL = SPI_CPOL_High;
    SPI_InitDef.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitDef.SPI_NSS = SPI_NSS_Soft;
    SPI_InitDef.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitDef.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitDef.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1,&SPI_InitDef);

    SPI_Cmd(SPI1,ENABLE);
}

//***************************************************//
//  功能描述: 
//  
//  参数: 无
//  
//  返回值: TRUE / FALSE
//  
//  说明: 无
//  
//***************************************************//
void u8g2_Init()
{
    OLED_Init();
    #if(OLED_TYPE == SSD1306)
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2_Data, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay); 
    #elif(OLED_TYPE == SH1106)
    u8g2_Setup_sh1106_128x64_noname_f(&u8g2_Data, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay); 
    #else
    #warning "显示设备未适配"
    #endif
	u8g2_InitDisplay(&u8g2_Data);
	u8g2_SetPowerSave(&u8g2_Data, 0);
    u8g2_ClearDisplay(&u8g2_Data);
}





void u8g2_Logo(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1);  // Transparent
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);

    u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(u8g2, 1,54,"Design By LHY");	
}


void Display_FreeRTOS_Logo()
{
    u8g2_ClearBuffer(&u8g2_Data);
    u8g2_DrawXBM(&u8g2_Data,0,8,128,48,FreeRTOS_Logo);
    u8g2_SendBuffer(&u8g2_Data);
    Delay_ms(500);
    u8g2_ClearBuffer(&u8g2_Data);
}


void Display_U8g2_Logo()
{

    u8g2_Logo(&u8g2_Data);
    u8g2_SendBuffer(&u8g2_Data); 
    Delay_ms(500);  
    u8g2_ClearBuffer(&u8g2_Data);
}

void DispalyUTF_8Strings()
{   
    u8g2_ClearBuffer(&u8g2_Data);
    u8g2_SetFontMode(&u8g2_Data,0);
    u8g2_SetFont(&u8g2_Data, u8g2_font_6x12_tr);
    u8g2_DrawUTF8(&u8g2_Data,10,10,"STM32 U8g2");

    u8g2_DrawButtonUTF8(&u8g2_Data,40,40,U8G2_BTN_HCENTER|U8G2_BTN_BW1|U8G2_BTN_SHADOW1,0,1,1,"OK");
    u8g2_SendBuffer(&u8g2_Data); 
     u8g2_ClearBuffer(&u8g2_Data);
    Delay_ms(500);
    u8g2_DrawButtonUTF8(&u8g2_Data,41,41,U8G2_BTN_HCENTER|U8G2_BTN_BW1,0,1,1,"FreeRTOS");
    u8g2_SendBuffer(&u8g2_Data);   
    Delay_ms(500);
}

void Mov(u16 x,u16 y)
{
    char Data[6];

    // u8g2_ClearBuffer(&u8g2_Data);
    // u8g2_DrawButtonUTF8(&u8g2_Data,40,40,U8G2_BTN_HCENTER|U8G2_BTN_BW1|U8G2_BTN_SHADOW1,0,1,1,"OK");
    u8g2_ClearBuffer(&u8g2_Data);
    memset(Data,0x00,sizeof(Data)-1);
    
    sprintf(Data+1,"%d",x);
    Data[0]='x';
    u8g2_DrawUTF8(&u8g2_Data,x,y+8,Data);
    sprintf(Data+1,"%d",y);
    Data[0]='y';
    u8g2_DrawUTF8(&u8g2_Data,x,y+16,Data);
    u8g2_SendBuffer(&u8g2_Data); 


}


void Power_On()
{
    // static uint8_t x=1,y=0,z=0;
    // static uint8_t px=1,py=0;

    // if(x <= 31)
    // {
    //     u8g2_DrawHLine(&u8g2_Data,62-x*2,31,(x+1)*4);
    //     u8g2_DrawHLine(&u8g2_Data,62-x*2,32,(x+1)*4);
    //     x++;
    // }
    // else
    // {
    //     px = 63*FastCos(DEGTORAD(y));
    //     py = 31*FastSin(DEGTORAD(y));

    //     if(y<90)
    //     {
    //         y++;
    //         u8g2_DrawVLine(&u8g2_Data,&u8g2_Data,63,31,63-px,31+py);
    //         u8g2_DrawVLine(&u8g2_Data,&u8g2_Data,63,31,63-px,31-py);
    //         u8g2_DrawVLine(&u8g2_Data,&u8g2_Data,63,31,63+px,31+py);
    //         u8g2_DrawVLine(&u8g2_Data,&u8g2_Data,63,31,63+px,31-py);
    //     }
    //     else
    //     {
    //         if(z<63)
    //         {
    //             z++;
    //             u8g2_DrawVLine(&u8g2_Data,63-z,0,63);
    //             u8g2_DrawVLine(&u8g2_Data,64+z,0,63);

    //             if( z > 11 )
    //             {
    //                 u8g2_SetFont(&u8g2_Data, u8g2_font_streamline_design_t);
    //                 u8g2_DrawGlyph(&u8g2_Data,51,84-z,0x011F);
    //             }
    //         }
    //         else
    //         {
    //             x=1;
    //             y=0;
    //             z=0;
    //             GUI_Info.Index = Main_ui;
    //         }
    //     }
    // }
}


#define  SIZE  6
// _3D Cube[SIZE]=
// {
//     {0,0,0},
//     {8,0,0},
//     {0,8,0},
//     {8,8,0},
    
//     {0,0,8},
//     {8,0,8},
//     {0,8,8},
//     {8,8,8}
// };
_3D Cube[SIZE]=
{
    {0,6,0},
    {6,6,0},
    {6,6,6},
    {0,6,6},
    
    {3,12,3},
    {3,0,3},
};

/***************************************
函数: OrtProject
功能: 正射投影(Orthographic projection)
***************************************/
_2D OrtProject(_3D  Space)
{
    _2D  Screen;
    //---------------
    Screen.x = (int)Space.x;
    Screen.y = (int)Space.y;
    //---------------
    return Screen;
}



//基于透视投影的标准模型
#define FOCAL_DISTANCE 128 //视点到视平面的距离
int  XOrigin = 63;
int  YOrigin = 33;
/***************************************
函数: PerProject
功能: 透视投影(Perspective projection)
说明: 1.又称为中心投影法
      2.XOrigin,YOrigin为投影后的图形中心的屏幕坐标
***************************************/
_2D PerProject(_3D  Space)
{
    _2D  Screen;
    //-------------------
    if(Space.z==0)Space.z=1; //被除数不能为零
    Screen.x = (int)( FOCAL_DISTANCE * Space.x / (Space.z + FOCAL_DISTANCE) ) + XOrigin;
    Screen.y = (int)( FOCAL_DISTANCE * Space.y / (Space.z + FOCAL_DISTANCE) ) + YOrigin;
    //-------------------
    return Screen;
}


void RotateCube2( float ax, float ay, float az )
{
    float  gmat[4][4];
    u8  i;
    _3D  temp;
    _2D  Cube_Dis[SIZE];
    // _2D  Triangle_Dis[3];
    //---------------------
    Identity_3D(gmat);			//单位矩阵化
    Translate_3D(gmat,-3,-3,-3);
    Scale_3D(gmat,4,4,4);
    Rotate_3D(gmat,ax,ay,az);
    Translate_3D(gmat,0,0,40);
    //---------------------
    //---------------------
    for(i=0;i<SIZE;i++)
    {
        temp = VEC_MultMatrix(Cube[i],gmat);
        Cube_Dis[i] = PerProject(temp);
    }

    //---------------------
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[0].x,Cube_Dis[0].y,Cube_Dis[1].x,Cube_Dis[1].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[0].x,Cube_Dis[0].y,Cube_Dis[2].x,Cube_Dis[2].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[3].x,Cube_Dis[3].y,Cube_Dis[1].x,Cube_Dis[1].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[3].x,Cube_Dis[3].y,Cube_Dis[2].x,Cube_Dis[2].y);
    // //------------------------------------------
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[0+4].x,Cube_Dis[0+4].y,Cube_Dis[1+4].x,Cube_Dis[1+4].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[0+4].x,Cube_Dis[0+4].y,Cube_Dis[2+4].x,Cube_Dis[2+4].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[3+4].x,Cube_Dis[3+4].y,Cube_Dis[1+4].x,Cube_Dis[1+4].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[3+4].x,Cube_Dis[3+4].y,Cube_Dis[2+4].x,Cube_Dis[2+4].y);
    // //------------------------------------------
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[0].x,Cube_Dis[0].y,Cube_Dis[0+4].x,Cube_Dis[0+4].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[1].x,Cube_Dis[1].y,Cube_Dis[1+4].x,Cube_Dis[1+4].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[2].x,Cube_Dis[2].y,Cube_Dis[2+4].x,Cube_Dis[2+4].y);
    // u8g2_DrawLine(&u8g2_Data,Cube_Dis[3].x,Cube_Dis[3].y,Cube_Dis[3+4].x,Cube_Dis[3+4].y);


    u8g2_DrawLine(&u8g2_Data,Cube_Dis[0].x,Cube_Dis[0].y,Cube_Dis[1].x,Cube_Dis[1].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[1].x,Cube_Dis[1].y,Cube_Dis[2].x,Cube_Dis[2].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[2].x,Cube_Dis[2].y,Cube_Dis[3].x,Cube_Dis[3].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[3].x,Cube_Dis[3].y,Cube_Dis[0].x,Cube_Dis[0].y);
    //------------------------------------------
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[4].x,Cube_Dis[4].y,Cube_Dis[0].x,Cube_Dis[0].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[4].x,Cube_Dis[4].y,Cube_Dis[1].x,Cube_Dis[1].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[4].x,Cube_Dis[4].y,Cube_Dis[2].x,Cube_Dis[2].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[4].x,Cube_Dis[4].y,Cube_Dis[3].x,Cube_Dis[3].y);
    // //------------------------------------------
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[5].x,Cube_Dis[5].y,Cube_Dis[0].x,Cube_Dis[0].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[5].x,Cube_Dis[5].y,Cube_Dis[1].x,Cube_Dis[1].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[5].x,Cube_Dis[5].y,Cube_Dis[2].x,Cube_Dis[2].y);
    u8g2_DrawLine(&u8g2_Data,Cube_Dis[5].x,Cube_Dis[5].y,Cube_Dis[3].x,Cube_Dis[3].y);

}

float turn;
u8 ops = 0;
void u8g2_Task()
{
    u8g2_ClearBuffer(&u8g2_Data);

    // GUI_Info.KeyInfo = GetKeyState();

    // GUI_Info.UI_List[GUI_Info.Index]();

    u8g2_DrawFrame(&u8g2_Data,0,0,128,64);
    u8g2_DrawCircle(&u8g2_Data,63,42,20,U8G2_DRAW_ALL);
    RotateCube2(0,turn,0);
    u8g2_SetFont(&u8g2_Data, u8g2_font_cu12_t_symbols);
    // u8g2_DrawUTF8(&u8g2_Data,10,10,"Preface");
    ops = 16;
    if(ops <= 16)
    {
        u8g2_DrawGlyph(&u8g2_Data,10,ops,0x50);
        u8g2_DrawGlyph(&u8g2_Data,23,ops,0x72);
        u8g2_DrawGlyph(&u8g2_Data,32,ops,0x65);
        u8g2_DrawGlyph(&u8g2_Data,42,ops,0x66);
        u8g2_DrawGlyph(&u8g2_Data,52,ops,0x61);
        u8g2_DrawGlyph(&u8g2_Data,62,ops,0x63);
        u8g2_DrawGlyph(&u8g2_Data,72,ops,0x65);
        // ops++;
    }
    else
    {
        ops = 0xff;
    }

    turn+=1;    
    if(turn>=360)
        turn=0;
    if(flag == 0)
    {
        if(turn<=180)
        {
            Cube[4].y = 6+turn/30;
            Cube[5].y = 6-turn/30;
        }
        else
        {
            // Cube[4].y = 12-(turn-180)/30;
            // Cube[5].y = (turn-180)/30;
            flag = 1;
        }
    }

        Delay_ms(5);

    u8g2_SendBuffer(&u8g2_Data);
}



