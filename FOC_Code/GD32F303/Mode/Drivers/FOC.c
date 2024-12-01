#include "DataType.h"
#include "gd32f30x.h"
#include <stdio.h>
#include "FOC.h"
#include "AS5600.h"
#include "PID.h"
#include <math.h>
#include "Timer.h"
#include "LED.h"


static PID_t PositionPID;
static u16  TIM_PeriodVal = TIME_CLK*1000/CLK_DIV/PWM_FRQUENCE;





void PWM_Init()
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(FOC_TIMER);

    //  ���PWM�ֱ����Ƿ��㹻���뱣��PWMƵ�ʲ��������£�
    //  ���Լ�С CLK_DIV ��ֵ������
    //  ��֤��100�ı���
    if( (TIM_PeriodVal%100) != 0)
    {
      while (1)
      {
        TIM_PeriodVal ++;
      }
    }

    /* FOC_TIMER configuration */
    timer_initpara.prescaler         = CLK_DIV;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = TIM_PeriodVal;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(FOC_TIMER,&timer_initpara);

    /* CH0,CH1 and CH2 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(FOC_TIMER,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(FOC_TIMER,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(FOC_TIMER,TIMER_CH_2,&timer_ocintpara);

    /* CH0 configuration in PWM mode0,duty cycle 25% */
    timer_channel_output_pulse_value_config(FOC_TIMER,TIMER_CH_0,3999);
    timer_channel_output_mode_config(FOC_TIMER,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(FOC_TIMER,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 50% */
    timer_channel_output_pulse_value_config(FOC_TIMER,TIMER_CH_1,7999);
    timer_channel_output_mode_config(FOC_TIMER,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(FOC_TIMER,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0,duty cycle 75% */
    timer_channel_output_pulse_value_config(FOC_TIMER,TIMER_CH_2,11999);
    timer_channel_output_mode_config(FOC_TIMER,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(FOC_TIMER,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(FOC_TIMER);
    /* auto-reload preload enable */
    timer_enable(FOC_TIMER);


    // FOC_ENABLE;
    PID_Init(&PositionPID);

}

// ����ռ�ձ�(%)
// PluseWide = 10 : 10%
void PWM_SetDuty(u8 PluseWideA,u8 PluseWideB,u8 PluseWideC)
{
    timer_channel_output_pulse_value_config(FOC_TIMER,TIMER_CH_0,(TIM_PeriodVal*PluseWideA/100-1));
    timer_channel_output_pulse_value_config(FOC_TIMER,TIMER_CH_1,(TIM_PeriodVal*PluseWideB/100-1));
    timer_channel_output_pulse_value_config(FOC_TIMER,TIMER_CH_2,(TIM_PeriodVal*PluseWideC/100-1));
}

//���Ƕ� = ����Ƕ�*������
float ElectricalAngle(float physics_angle, int pole_pairs) 
{
  return (physics_angle * (float)pole_pairs);
}


// ���ƽǶ�[0 - 360]
float AngleLimit(float Input) 
{
    float Tmp;
    Tmp = (float)fmod(Input,360.0);
    if( Tmp < 0.0f)
    {
       Tmp += 360.0f;
    }
    return Tmp;
}


//��ͨ�˲���
float Proportion = 0.8;
float LowPass_Filter(float x)
{
  static float y_prev = 0.0f;
	float y = (1.0f-Proportion)*y_prev + Proportion*x;
	y_prev=y;
	return y;
}

//����ƽ���˲���
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

    // ��0��360��֮��ĽǶȹ�һ��
    // ֻ����ʹ�� _sin�� _cos ���ƺ���ʱ����Ҫ
    angle_el = AngleLimit(angle_el);
    // ����PWM����
    // ���ɿ�+�����˱任
    SinVal = FastSin(DEGTORAD(angle_el));
    CosVal = FastCos(DEGTORAD(angle_el));

    // ���ɿ˱�
    Ualpha = Ud * CosVal - Uq * SinVal; 
    Ubeta =  Ud * SinVal + Uq * CosVal; 

    // �����˱任
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

      // �ҵ�����Ŀǰ����������
      int sector = floor(angle_el / 60.0f) + 1;
      // ����ռ�ձ�
      float T1 = SQRT_3*FastSin(DEGTORAD(sector*60.0f - angle_el))* Uout;
      float T2 = SQRT_3*FastSin(DEGTORAD(angle_el - (sector-1.0)*60.0f)) * Uout;
      // �����汾
      // �Ե�ѹ��ԴΪ����/2
      float T0 = 1 - T1 - T2;
      // �͵�Դ��ѹ������0
      //float T0 = 0;

      // ����ռ�ձȣ�ʱ�䣩
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
         // ���ܵĴ���״̬
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }

    // �������ѹ������
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
	//����޷�
	Ua = ValueLimit(Ua,0.0f,VCC_MOTOR);
	Ub = ValueLimit(Ub,0.0f,VCC_MOTOR);
	Uc = ValueLimit(Uc,0.0f,VCC_MOTOR);

  PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
  PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
  PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));

}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	//��������
	Uq = ValueLimit(Uq,-VCC_MOTOR/2,VCC_MOTOR/2);
	
	//�Ƕȹ�һ��
	angle_el = AngleLimit(angle_el);
	
	//���Ƕ�
	angle_el = ElectricalAngle(angle_el,POLE_PAIR);
	
	//park��任
	Ualpha = -Uq * FastSin(DEGTORAD(angle_el)) + Ud * FastCos(DEGTORAD(angle_el));
	Ubate  =  Uq * FastCos(DEGTORAD(angle_el)) + Ud * FastSin(DEGTORAD(angle_el));
	
	//clarke��任
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




// ��ʱ��3�жϷ������  10ms
void TIM3_IRQHandler(void) // TIM3�ж�
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


// #define volatge_high 	10.0f				//��ѹ����ֵ
// #define Udc 			10.0f				//ĸ�ߵ�ѹ
// #define sqrt3			1.732				//����3
// #define polePairs 	 	7 					// ����ļ�����
// // #define _2PI 	 		6.2831853 			// 2PI
// // #define _PI 	 		3.1415927 			// PI
// // #define _3PI_2			4.71238898038		//PI/3

// float  zero = 0.0f;							//���Ƕ�
// float angle_err=0.0f,angle_last_err=0.0f,angle_out=0.0f,p=15.0f,d=2.0f;
// void setPhaseVoltage(float Uq, float Ud, float angle_el);
// //��ֵ���ƺ���
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
 
// // ��Ƕ� = ��е�Ƕ� * ������
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


// //����
// void angle_init(float jixie_angle)
// {
// 	setPhaseVoltage(3,0,_3PI_2);
	
// 	Delay_ms(1000);
// 	Delay_ms(1000);
// 	Delay_ms(1000);
	
// 	zero = _electricalAngle(jixie_angle);
	
// 	setPhaseVoltage(0,0,_3PI_2);
// 	// printf("��ʼ�����\r\n");
// 	// printf("���λ�Ƕ�:	%lf\r\n",zero);
// }

// // �ѽǶ�ֵ��һ���� [0, 2pi]
// float Angle_limit(float angle)
// {
//     float a = fmod(angle, _2PI); // fmod()�������ڸ�������ȡ������
//     return a >= 0.0f ? a : (a + _2PI);
// }

// float pwm_a=0,pwm_b=0,pwm_c=0;
// void setpwm(float Ua,float Ub,float Uc)
// {
// 	//����޷�
// 	Ua = limit(Ua,0.0f,volatge_high);
// 	Ub = limit(Ub,0.0f,volatge_high);
// 	Uc = limit(Uc,0.0f,volatge_high);
	
// 	//PWM�޷�
// 	// pwm_a = limit(Ua / Udc , 0.0f , 1.0f);
// 	// pwm_b = limit(Ub / Udc , 0.0f , 1.0f);
// 	// pwm_c = limit(Uc / Udc , 0.0f , 1.0f);
	
// 	//PWMд��
//     PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
//     PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
//     PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));
// 	//printf("%lf,%lf,%lf\n",pwm_a * 2600,pwm_b * 2600,pwm_c * 2600);
// }

// // FOC���ĺ���������Uq��Ud�͵�Ƕȣ������·PWM
// float Ualpha=0.0f,Ubate=0.0f;
// float Ua=0.0f,Ub=0.0f,Uc=0.0f;
// void setPhaseVoltage(float Uq, float Ud, float angle_el)
// {
// 	//��������
// 	Uq = limit(Uq,-Udc/2,Udc/2);
	
// 	//�Ƕȹ�һ��
// 	angle_el = Angle_limit(angle_el);
	
// 	//���Ƕ�
// 	angle_el = _electricalAngle(angle_el);
	
// 	//park��任
// 	Ualpha = -Uq * FastSin(angle_el) + Ud * FastCos(angle_el);
// 	Ubate  =  Uq * FastCos(angle_el) + Ud * FastSin(angle_el);
	
// 	//clarke��任
// 	Ua = Ualpha + Udc/2;
// 	Ub = (sqrt3 * Ubate - Ualpha)/2 + Udc/2;
// 	Uc = (-sqrt3 * Ubate - Ualpha)/2 + Udc/2;
	
// 	//printf("%lf,%lf,%lf\n",Ua,Ub,Uc);
// 	setpwm(Ua,Ub,Uc);
// }

// // ��ʱ��3�жϷ������  10ms
// void TIM3_IRQHandler(void) // TIM3�ж�
// {
// 	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
// 	{
// 		angle = AS5600_Angle(ANGLE_TURN_MODE);
// 		local_out = angle_pid(angle,50);
// 		setPhaseVoltage(local_out,0,angle);

// 		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

// 	}
// }






