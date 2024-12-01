#include <stdio.h>
#include <inttypes.h>
#include "DataType.h"
#include "PID.h"



// static PID_t pid;


//***************************************************//
//  ��������: PID������ʼ��
//  
//  ����: ��
//  
//  ����ֵ: ��
//  
//  ˵��: ��
//  
//***************************************************//
void PID_Init(PID_t *PID_Data)
{
	PID_Data->TargetValue = 0.0;
	PID_Data->IntegralValue = 0.0;
	PID_Data->OutValue = 0.0;
	PID_Data->err = 0.0;
	PID_Data->err_last = 0.0;
	PID_Data->err_next = 0.0;
	//�����������ֵ���Ӷ��ﵽ���Ч��
	PID_Data->Kp = 0.0;
	PID_Data->Ki = 0.0; 
	PID_Data->Kd = 0.0;
}

//***************************************************//
//  ��������: �޸�PIDϵ��
//  
//  ����: P I D ϵ��
//  
//  ����ֵ: ��
//  
//  ˵��: ��
//  
//***************************************************//
void PID_Change_Kp(PID_t *PID_Data, float k)
{
	PID_Data->Kp = k;
}

void PID_Change_Ki(PID_t *PID_Data, float k)
{
	PID_Data->Ki = k; 
}

void PID_Change_Kd(PID_t *PID_Data, float k)
{
	PID_Data->Kd = k;
}
void PID_SetTarget(PID_t *PID_Data, float k)
{
	PID_Data->TargetValue = k;
}
//***************************************************//
//  ��������: PID������
//  
//  ����:�趨ֵ
//  
//  ����ֵ: ��ǰֵ
//  
//  ˵��: ��
//  
//***************************************************//
float PID_Process(PID_t *PID_Data,float Actual)
{
	PID_Data->err = PID_Data->TargetValue - Actual;

	PID_Data->IntegralValue += PID_Data->err;

    PID_Data->OutValue = PID_Data->Kp*(PID_Data->err) 
					   + PID_Data->Ki*(PID_Data->IntegralValue) 
					   + PID_Data->Kd*(PID_Data->err - PID_Data->err_last);

	PID_Data->err_last = PID_Data->err;

	return PID_Data->OutValue;
}
