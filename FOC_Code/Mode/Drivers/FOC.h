#ifndef FOC_H
#define FOC_H

// 满周期计数值  = TIME_CLK*1000/CLK_DIV/PWM_FRQUENCE
// 建议不低于20khz，否则电机噪声大
#define FOC_TIMER           (TIM1)
#define TIME_CLK            (72)//定时器时钟:Mhz
#define CLK_DIV             (2)//能被 TIME_CLK 整除,
#define PWM_FRQUENCE        (120)//Khz

//电机极对数
#define POLE_PAIR	(7)

//最大电压
#define VCC_MOTOR	(10.0f)

//PWM 通道数：3或6路
#define PWM_3_NUM           (3)
#define PWM_6_NUM           (6)
#define PWM_CHANNEL         (PWM_3_NUM)

#define FOC_DISABLE     GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define FOC_ENABLE      GPIO_SetBits(GPIOB,GPIO_Pin_12)

#define SQRT_3      (1.7320508075f)//sqrt(3)/
#define SQRT_3_2    (0.8660254037f)//sqrt(3)/2

#define ValueLimit(Val,Min,Max) ((Val)<(Min)?(Min):((Val)>(Max)?(Max):(Val)))

void PWM_Init(void);
void FOC_GPIO_Init(void);
void FOC_main(void);
void PWM_Task(void);
void PWM_SetDuty(u8 PluseWideA,u8 PluseWideB,u8 PluseWideC);
void Foc_CTL(void);
void FOC_TickTask(void);
#endif

