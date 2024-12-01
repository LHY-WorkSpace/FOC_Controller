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
static u16  TIM_PeriodVal = TIME_CLK*1000/CLK_DIV/PWM_FRQUENCE;




// PWM_1:PA8-主  PB13-互补
// PWM_2:PA11-主  PB14-互补
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

	PWM_IO.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_10;	
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
  TIM_TimeBaseInit(FOC_TIMER,&TIM_TimeBaseInitsruc);


  TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
#if(PWM_CHANNEL == PWM_3_NUM)
  TIM_OCInit.TIM_OutputNState = TIM_OutputNState_Disable;
#else 
  TIM_OCInit.TIM_OutputNState = TIM_OutputNState_Enable;
#endif  
  TIM_OCInit.TIM_Pulse = 0;
  TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;
#if(PWM_CHANNEL == PWM_3_NUM)
  TIM_OCInit.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#else 
  TIM_OCInit.TIM_OCNPolarity = TIM_OCPolarity_High;//相同时为互补反相
#endif 
  TIM_OCInit.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInit.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  TIM_OCInit.TIM_OCIdleState = TIM_OCIdleState_Reset;//死区后输出的极性
  TIM_OCInit.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//死区后输出的极性
#if(PWM_CHANNEL == PWM_6_NUM)
  TIM_BDTRInitTypeDef TIM_BDTRInit;
  TIM_BDTRInit.TIM_OSSRState = TIM_OSSRState_Disable;
  TIM_BDTRInit.TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInit.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInit.TIM_DeadTime = 0x40;// 每增加0x40，时间增加500ns
  TIM_BDTRInit.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInit.TIM_BreakPolarity = TIM_BreakPolarity_High;//死区后输出的极性
  TIM_BDTRInit.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(FOC_TIMER,&TIM_BDTRInit);
#endif

  TIM_OC1Init(FOC_TIMER,&TIM_OCInit);
  TIM_OC4Init(FOC_TIMER,&TIM_OCInit);
  TIM_OC3Init(FOC_TIMER,&TIM_OCInit);

  TIM_OC1FastConfig(FOC_TIMER,TIM_OCFast_Enable);
  TIM_OC4FastConfig(FOC_TIMER,TIM_OCFast_Enable);
  TIM_OC3FastConfig(FOC_TIMER,TIM_OCFast_Enable);

  TIM_OC1PreloadConfig(FOC_TIMER, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(FOC_TIMER, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(FOC_TIMER, TIM_OCPreload_Enable);

  TIM_CtrlPWMOutputs(FOC_TIMER,ENABLE);
  TIM_ARRPreloadConfig(FOC_TIMER,ENABLE);
  FOC_ENABLE;
  TIM_Cmd(FOC_TIMER,ENABLE);
  PID_Init(&PositionPID);

}

// 设置占空比(%)
// PluseWide = 10 : 10%
// 效果不对时，可调整 Ux和CCRx的对应关系
void PWM_SetDuty(u8 PluseWideA,u8 PluseWideB,u8 PluseWideC)
{
  FOC_TIMER->CCR1 = TIM_PeriodVal*PluseWideA/100-1;//PA8
  FOC_TIMER->CCR4 = TIM_PeriodVal*PluseWideB/100-1;//PA11
  FOC_TIMER->CCR3 = TIM_PeriodVal*PluseWideC/100-1;//PA10
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


//低通滤波器
float Proportion = 0.8;
float LowPass_Filter(float x)
{
  static float y_prev = 0.0f;
	float y = (1.0f-Proportion)*y_prev + Proportion*x;
	y_prev=y;
	return y;
}

//递推平均滤波器
float Avg_Filter(float x)
{
  static float PreVal = 0.0f;
  float RetVal;

  RetVal = (PreVal+x)/2;
  PreVal = x;
  return RetVal;
}




#if 1
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

    // printf("FOC:%.1f,%.1f,%.1f\n",Ua*100,Ub*100,Uc*100);
    PWM_SetDuty((uint8_t)(Ua*100/VCC_MOTOR),(uint8_t)(Ub*100/VCC_MOTOR),(uint8_t)(Uc*100/VCC_MOTOR));

}


void SVPWM_CTL(float Uq, float Ud,float angle_el) 
{
    float Uout;
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
    // Ua = Ta*VCC_MOTOR;
    // Ub = Tb*VCC_MOTOR;
    // Uc = Tc*VCC_MOTOR;
    // printf("FOC:%.1f,%.1f,%.1f\n",Ua*100,Ub*100,Uc*100);
    // PWM_SetDuty((uint8_t)(Ua*100/VCC_MOTOR),(uint8_t)(Ub*100/VCC_MOTOR),(uint8_t)(Uc*100/VCC_MOTOR));
    PWM_SetDuty((uint8_t)(Ta*100),(uint8_t)(Tb*100),(uint8_t)(Tc*100));
}

void FocOpenLoop_Speed(float Speed)
{
  float UqVal = 1.0;
  static float angtmp = 0.0f;
  angtmp = AngleLimit(angtmp+Speed);  
  SIN_CTL(UqVal,0,ElectricalAngle(angtmp,POLE_PAIR));
  Delay_ms(5);
}
u8 flag = 2;
float G_P = 0.04;
float G_I = 0.0;
float G_D = 0.3;
float Tarang = 180.0;
float MAX = 1.5;
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
  // Angle = Avg_Filter(Angle);
  // printf("FOC:%.1f,%.1f\n",Target,Angle);
  angtmp = AngleLimit(Angle);

  Angle = PID_Process(&PositionPID,Angle);


  Angle =  ValueLimit(Angle,-MAX,MAX);
  UqTmp = ElectricalAngle(angtmp,POLE_PAIR)*DIR;
  UqTmp = AngleLimit(UqTmp);

  Angle = LowPass_Filter(Angle);
  printf("FOC:%.1f,%.1f\n",Angle,UqTmp);
  SIN_CTL(Angle,0,UqTmp);
  // SVPWM_CTL(Angle,0,UqTmp);

}

void Foc_CTL()
{
  FocCloseLoop_Position(Tarang);
  // FocOpenLoop_Speed(Tarang);
}

#endif

#if 0
float Ua=0.0f,Ub=0.0f,Uc=0.0f;
void Foc_CTL()
{
 Delay_ms(2);
}
void setpwm(float Ua,float Ub,float Uc)
{
	//输出限幅
	Ua = ValueLimit(Ua,0.0f,VCC_MOTOR);
	Ub = ValueLimit(Ub,0.0f,VCC_MOTOR);
	Uc = ValueLimit(Uc,0.0f,VCC_MOTOR);

  PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
  PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
  PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));

}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	//力矩限制
	Uq = ValueLimit(Uq,-VCC_MOTOR/2,VCC_MOTOR/2);
	
	//角度归一化
	angle_el = AngleLimit(angle_el);
	
	//求电角度
	angle_el = ElectricalAngle(angle_el,POLE_PAIR);
	
	//park逆变换
	Ualpha = -Uq * FastSin(DEGTORAD(angle_el)) + Ud * FastCos(DEGTORAD(angle_el));
	Ubate  =  Uq * FastCos(DEGTORAD(angle_el)) + Ud * FastSin(DEGTORAD(angle_el));
	
	//clarke逆变换
	Ua = Ualpha + VCC_MOTOR/2;
	Ub = (_SQRT3 * Ubate - Ualpha)/2 + VCC_MOTOR/2;
	Uc = (-_SQRT3 * Ubate - Ualpha)/2 + VCC_MOTOR/2;
	
	//printf("%lf,%lf,%lf\n",Ua,Ub,Uc);
	setpwm(Ua,Ub,Uc);
}

float local_out = 0.0;
float angle_err=0.0f,angle_last_err=0.0f,angle_out=0.0f,p=15.0f,d=2.0f;
float angle_pid(float now_angle,float tar_angle)
{
	angle_err = tar_angle-now_angle;
	angle_out=p*angle_err+d*(angle_err-angle_last_err);
	angle_out = ValueLimit(angle_out,-6,6);
    angle_last_err=angle_err;
    return angle_out;
}

void  angle_clear(void)
{
	angle_err=0;
	angle_last_err=0;
	angle_out=0;
}




// 定时器3中断服务程序  10ms
void TIM3_IRQHandler(void) // TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		angle = AS5600_Angle(ANGLE_TURN_MODE);
		// all_angle = angle_get_all(angle);
		local_out = angle_pid(angle,50);
		setPhaseVoltage(local_out,0,angle);

		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	}
}
#endif


// #define volatge_high 	10.0f				//电压限制值
// #define Udc 			10.0f				//母线电压
// #define sqrt3			1.732				//根号3
// #define polePairs 	 	7 					// 电机的极对数
// // #define _2PI 	 		6.2831853 			// 2PI
// // #define _PI 	 		3.1415927 			// PI
// // #define _3PI_2			4.71238898038		//PI/3

// float  zero = 0.0f;							//零电角度
// float angle_err=0.0f,angle_last_err=0.0f,angle_out=0.0f,p=15.0f,d=2.0f;
// void setPhaseVoltage(float Uq, float Ud, float angle_el);
// //幅值限制函数
// float limit(float in_vo,float low,float high)
// {
// 	if(in_vo>=high)
// 		in_vo=high;
// 	else if(in_vo<=low)
// 		in_vo=low;
// 	else
// 		in_vo=in_vo;
// 	return in_vo;
// }
 
// // 电角度 = 机械角度 * 极对数
// float _electricalAngle(float jixie_angle)
// {
//     return (jixie_angle * polePairs - zero);
// }

// float angle_pid(float now_angle,float tar_angle)
// {
// 	angle_err = tar_angle-now_angle;
// 	angle_out=p*angle_err+d*(angle_err-angle_last_err);
// 	//angle_control(angle_out);
// 	angle_out = limit(angle_out,-6,6);
//     angle_last_err=angle_err;
//     return angle_out;
// }

// void  angle_clear(void)
// {
// 	angle_err=0;
// 	angle_last_err=0;
// 	angle_out=0;
// }


// //矫正
// void angle_init(float jixie_angle)
// {
// 	setPhaseVoltage(3,0,_3PI_2);
	
// 	Delay_ms(1000);
// 	Delay_ms(1000);
// 	Delay_ms(1000);
	
// 	zero = _electricalAngle(jixie_angle);
	
// 	setPhaseVoltage(0,0,_3PI_2);
// 	// printf("初始化完成\r\n");
// 	// printf("零电位角度:	%lf\r\n",zero);
// }

// // 把角度值归一化在 [0, 2pi]
// float Angle_limit(float angle)
// {
//     float a = fmod(angle, _2PI); // fmod()函数用于浮点数的取余运算
//     return a >= 0.0f ? a : (a + _2PI);
// }

// float pwm_a=0,pwm_b=0,pwm_c=0;
// void setpwm(float Ua,float Ub,float Uc)
// {
// 	//输出限幅
// 	Ua = limit(Ua,0.0f,volatge_high);
// 	Ub = limit(Ub,0.0f,volatge_high);
// 	Uc = limit(Uc,0.0f,volatge_high);
	
// 	//PWM限幅
// 	// pwm_a = limit(Ua / Udc , 0.0f , 1.0f);
// 	// pwm_b = limit(Ub / Udc , 0.0f , 1.0f);
// 	// pwm_c = limit(Uc / Udc , 0.0f , 1.0f);
	
// 	//PWM写入
//     PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
//     PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
//     PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));
// 	//printf("%lf,%lf,%lf\n",pwm_a * 2600,pwm_b * 2600,pwm_c * 2600);
// }

// // FOC核心函数：输入Uq、Ud和电角度，输出三路PWM
// float Ualpha=0.0f,Ubate=0.0f;
// float Ua=0.0f,Ub=0.0f,Uc=0.0f;
// void setPhaseVoltage(float Uq, float Ud, float angle_el)
// {
// 	//力矩限制
// 	Uq = limit(Uq,-Udc/2,Udc/2);
	
// 	//角度归一化
// 	angle_el = Angle_limit(angle_el);
	
// 	//求电角度
// 	angle_el = _electricalAngle(angle_el);
	
// 	//park逆变换
// 	Ualpha = -Uq * FastSin(angle_el) + Ud * FastCos(angle_el);
// 	Ubate  =  Uq * FastCos(angle_el) + Ud * FastSin(angle_el);
	
// 	//clarke逆变换
// 	Ua = Ualpha + Udc/2;
// 	Ub = (sqrt3 * Ubate - Ualpha)/2 + Udc/2;
// 	Uc = (-sqrt3 * Ubate - Ualpha)/2 + Udc/2;
	
// 	//printf("%lf,%lf,%lf\n",Ua,Ub,Uc);
// 	setpwm(Ua,Ub,Uc);
// }

// // 定时器3中断服务程序  10ms
// void TIM3_IRQHandler(void) // TIM3中断
// {
// 	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
// 	{
// 		angle = AS5600_Angle(ANGLE_TURN_MODE);
// 		local_out = angle_pid(angle,50);
// 		setPhaseVoltage(local_out,0,angle);

// 		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

// 	}
// }






