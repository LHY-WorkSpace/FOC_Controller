#include "DataType.h"
#include "stm32f10x.h"
#include <stdio.h>
#include "FOC.h"
#include "AS5600.h"
#include "PID.h"
#include <math.h>
#include "Timer.h"
#include "LED.h"


static PID_t PositionPID;
static u16  TIM_PeriodVal = 72*1000/CLK_DIV/PWM_FRQUENCE;


// PWM_1:PA8-主  PB13-互补
// PWM_2:PA9-主  PB14-互补
// PWM_3:PA10-主 PB15-互补

void PWM_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

  GPIO_InitTypeDef PWM_IO;
	GPIO_InitTypeDef PWM_EN;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitsruc;
  TIM_OCInitTypeDef TIM_OCInit;
  TIM_BDTRInitTypeDef TIM_BDTRInit;
	
	PWM_IO.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;	
	PWM_IO.GPIO_Speed = GPIO_Speed_50MHz;
	PWM_IO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&PWM_IO);
  
#if(PWM_CHANNEL == 6)
	PWM_IO.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
	PWM_IO.GPIO_Speed = GPIO_Speed_50MHz;
	PWM_IO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&PWM_IO);
#endif

	PWM_EN.GPIO_Pin = GPIO_Pin_12;	
	PWM_EN.GPIO_Speed = GPIO_Speed_10MHz;
	PWM_EN.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&PWM_EN);

  //  检查PWM分辨率是否足够，想保持PWM频率不变的情况下，
  //  可以减小 CLK_DIV 的值来满足
  //  保证是100的倍数
  if( (TIM_PeriodVal%100) != 0)
  {
    while (1)
    {
      TIM_PeriodVal ++;
    }
  }

  TIM_TimeBaseInitsruc.TIM_Prescaler= CLK_DIV-1;
  TIM_TimeBaseInitsruc.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseInitsruc.TIM_Period=TIM_PeriodVal -  1;
  TIM_TimeBaseInitsruc.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitsruc.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitsruc);

#if(PWM_CHANNEL == 3)
  TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInit.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInit.TIM_Pulse = 0;
  TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInit.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInit.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInit.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
#else
  TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInit.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInit.TIM_Pulse = 0;
  TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInit.TIM_OCNPolarity = TIM_OCPolarity_High;//相同时为互补反相
  TIM_OCInit.TIM_OCIdleState = TIM_OCIdleState_Reset;//死区后输出的极性
  TIM_OCInit.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//死区后输出的极性

  TIM_BDTRInit.TIM_OSSRState = TIM_OSSRState_Disable;
  TIM_BDTRInit.TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInit.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInit.TIM_DeadTime = 0x40;// 每增加0x40，时间增加500ns
  TIM_BDTRInit.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInit.TIM_BreakPolarity = TIM_BreakPolarity_High;//死区后输出的极性
  TIM_BDTRInit.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1,&TIM_BDTRInit);
#endif

  TIM_OC1Init(TIM1,&TIM_OCInit);
  TIM_OC2Init(TIM1,&TIM_OCInit);
  TIM_OC3Init(TIM1,&TIM_OCInit);

  TIM_OC1FastConfig(TIM1,TIM_OCFast_Enable);
  TIM_OC2FastConfig(TIM1,TIM_OCFast_Enable);
  TIM_OC3FastConfig(TIM1,TIM_OCFast_Enable);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_CtrlPWMOutputs(TIM1,ENABLE);
  TIM_ARRPreloadConfig(TIM1,ENABLE);
  FOC_ENABLE;
  TIM_Cmd(TIM1,ENABLE);
  PID_Init(&PositionPID);

}

// 设置占空比(%)
// PluseWide = 10 : 10%
// 效果不对时，可调整 Ux和CCRx的对应关系
void PWM_SetDuty(u8 Phase ,u8 PluseWide)
{
    switch (Phase)
    {
        case UA_Phase:
            TIM1->CCR3 = TIM_PeriodVal/100*PluseWide-1;
            break;
         case UB_Phase:
            TIM1->CCR1 = TIM_PeriodVal/100*PluseWide-1;
            break;
        case UC_Phase:
            TIM1->CCR2 = TIM_PeriodVal/100*PluseWide-1;
            break;       
        default:
            break;
    }
}

//求电角度 = 物理角度*极对数
float ElectricalAngle(float physics_angle, int pole_pairs) 
{
  return (physics_angle * (float)pole_pairs);
}


// 限制角度[0 - 360]
float AngleLimit(float Input) 
{
    float Tmp;
    Tmp = fmod(Input,360.0);
    if( Tmp < 0.0)
    {
       Tmp += 360.0;
    }
    return Tmp;
}

void SIN_CTL(float Uq,float Ud, float angle_el) 
{
    float Ua,Ub,Uc;
    float Ualpha,Ubeta;
    float SinVal,CosVal;

    // 在0到360°之间的角度归一化
    // 只有在使用 _sin和 _cos 近似函数时才需要
    angle_el = AngleLimit(angle_el);
    // 正弦PWM调制
    // 逆派克+克拉克变换
    SinVal = FastSin(DEGTORAD(angle_el));
    CosVal = FastCos(DEGTORAD(angle_el));

    // 逆派克变
    Ualpha = Ud * CosVal - Uq * SinVal; 
    Ubeta =  Ud * SinVal + Uq * CosVal; 

    // 克拉克变换
    Ua = Ualpha + VCC_MOTOR/2;
    Ub = -0.5 * Ualpha  + SQRT_3_2 * Ubeta + VCC_MOTOR/2;
    Uc = -0.5 * Ualpha - SQRT_3_2 * Ubeta + VCC_MOTOR/2;

    PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
    PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
    PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));
}


void SVPWM_CTL(float Uq, float Ud,float angle_el) 
{
    float Uout;
    float Ua,Ub,Uc;
     if(Ud)
      { 
        Uout = _sqrt(Ud*Ud + Uq*Uq) / VCC_MOTOR;
        angle_el = AngleLimit(angle_el + atan2(Uq, Ud));
      }
      else
      {
        Uout = Uq / VCC_MOTOR;
        angle_el = AngleLimit(angle_el + 90.0f);
      }

      // 找到我们目前所处的象限
      int sector = floor(angle_el / 60.0f) + 1;
      // 计算占空比
      float T1 = SQRT_3*FastSin(DEGTORAD(sector*60.0f - angle_el))* Uout;
      float T2 = SQRT_3*FastSin(DEGTORAD(angle_el - (sector-1.0)*60.0f)) * Uout;
      // 两个版本
      // 以电压电源为中心/2
      float T0 = 1 - T1 - T2;
      // 低电源电压，拉到0
      //float T0 = 0;

      // 计算占空比（时间）
      float Ta,Tb,Tc; 
      switch(sector)
      {
        case 1:
          Ta = T1 + T2 + T0/2;
          Tb = T2 + T0/2;
          Tc = T0/2;
          break;
        case 2:
          Ta = T1 +  T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T0/2;
          break;
        case 3:
          Ta = T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T2 + T0/2;
          break;
        case 4:
          Ta = T0/2;
          Tb = T1+ T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 5:
          Ta = T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 6:
          Ta = T1 + T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T0/2;
          break;
        default:
         // 可能的错误状态
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }

    // 计算相电压和中心
    Ua = Ta*VCC_MOTOR;
    Ub = Tb*VCC_MOTOR;
    Uc = Tc*VCC_MOTOR;

    PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
    PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
    PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));
}

void FocOpenLoop_Speed(float Speed)
{
  float UqVal = 1.0;
  static float angtmp = 0.0f;
  angtmp = AngleLimit(angtmp+Speed);  
  SIN_CTL(UqVal,0,ElectricalAngle(angtmp,POLE_PAIR));
  Delay_ms(5);
}

float G_P = 0.03;
float G_I = 0.0;
float Tarang = 0.0;
float G_D = 0.0;
float MAX = 2.0;
void FocCloseLoop_Position(float Target)
{
  float angtmp = 0.0f;
  float Angle = 0.0f;  
  float UqTmp;
  float DIR = 1.0;


    PID_Change_Kp(&PositionPID,G_P);
    PID_Change_Ki(&PositionPID,G_I);
    PID_Change_Kd(&PositionPID,G_D);
    PID_SetTarget(&PositionPID,Target);




  Angle = AS5600_Angle(ANGLE_TURN_MODE);
  printf("FOC:%.1f,%.1f\n",Target,Angle);
  angtmp = AngleLimit(Angle);

  Angle = PID_Process(&PositionPID,Angle);


  Angle =  ValueLimit(Angle,-MAX,MAX);
  UqTmp = ElectricalAngle(angtmp,POLE_PAIR)*DIR;
  UqTmp = AngleLimit(UqTmp);

  SIN_CTL(Angle,0,UqTmp);
  // SVPWM_CTL(Angle,0,UqTmp);
  // Delay_ms(2);
}

void Foc_CTL()
{
  FocCloseLoop_Position(Tarang);
  // FocOpenLoop_Speed(Tarang);
}











