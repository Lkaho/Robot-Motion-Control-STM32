#ifndef __STEPDRIVER_H
#define __STEPDRIVER_H
#include "sys.h"
#include "stdlib.h"	

/********** ������ �˿ڶ��� **************
//DRIVER_DIR   PC0 
//DRIVER_OE    PC2 
//STEP_PULSE   PC7 (TIM8_CH2,LCD_RW)
******************************************/
#define DRIVER_DIR   PCout(0) // ��ת���� 
#define DRIVER_OE    PCout(2) // ʹ�ܽ� �͵�ƽ��Ч
#define RCR_VAL    255  //ÿ������RCR_VAL+1���Σ��ж�һ�Σ����ֵ��0~255�����ô�һЩ���Խ����ж�Ƶ��
#define DEVIDE     100   //���������ϸ����
#define STEP_ANG   1.8  //������������
typedef enum
{
	CW = 1,//�ߵ�ƽ˳ʱ��
	CCW = 0,//�͵�ƽ��ʱ��
}DIR_Type;//���з���

extern long target_pos;//�з��ŷ���
extern long current_pos;//�з��ŷ���

void Driver_Init(void);//��������ʼ��
void TIM8_OPM_RCR_Init(u16 arr,u16 psc);//TIM8_CH2 ���������+�ظ��������ܳ�ʼ��
void TIM8_Startup(u32 frequency);   //������ʱ��8
void Locate_Abs(long num,u32 frequency);//���Զ�λ����
void Locate_AbsAng(float ang, u32 frequency);

#endif


