#ifndef PID_H
#define PID_H

typedef struct 
{
	float TargetValue;//�����趨ֵ
	float err;//����ƫ��ֵ
	float err_next;//������һ��ƫ��ֵ
	float err_last;//��������ǰ��ƫ��ֵ
	float Kp, Ki, Kd;//������������֡�΢��ϵ��
	float IntegralValue;//����ֵ
	float OutValue;//���ֵ
}PID_t;



void PID_Init(PID_t *PID_Data);
void PID_Change_Kp(PID_t *PID_Data, float k);
void PID_Change_Ki(PID_t *PID_Data, float k);
void PID_Change_Kd(PID_t *PID_Data, float k);
void PID_SetTarget(PID_t *PID_Data, float k);
float PID_Process(PID_t *PID_Data,float Target);

#endif






