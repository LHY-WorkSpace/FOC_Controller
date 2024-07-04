#ifndef FOC_H
#define FOC_H
// �����ڼ���ֵ  = 72*1000/CLK_DIV/PWM_FRQUENCE
// ���鲻����20khz��������������
#define CLK_DIV             (2)//�ܱ�72����,
#define PWM_FRQUENCE        (120)//Khz
//���������
#define POLE_PAIR	(7)
//����ѹ
#define VCC_MOTOR	(10.0f)
//PWM ͨ������3��6·
#define PWM_CHANNEL      (3)

#define FOC_DISABLE     GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define FOC_ENABLE      GPIO_SetBits(GPIOB,GPIO_Pin_12)

#define SQRT_3      (1.7320508075f)//sqrt(3)/
#define SQRT_3_2    (0.8660254037f)//sqrt(3)/2

#define ValueLimit(Val,Min,Max) ((Val)<(Min)?(Min):((Val)>(Max)?(Max):(Val)))

typedef enum
{
    UA_Phase,
    UB_Phase,
    UC_Phase,
    U_PhaseMax,
}U_Phase_e;

void PWM_Init(void);
void FOC_GPIO_Init(void);
void FOC_main(void);
void PWM_Task(void);
void PWM_SetDuty(uint8_t Phase,uint8_t Value);
void Foc_CTL(void);
void FOC_TickTask(void);
#endif

