#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mbotLinuxUsart.h"
#include "adc.h"
#include "dac.h"
#include "exti.h"
#include "key.h"
#include "dir_gpio.h"
#include "timer.h"
#include "can.h"
#include "stepdriver.h"
#include "rs485.h"
//�����ı���

//���̿��Ʊ���(�Ŵ���100��)
int thr_val = 0;      //��λ�����͵�����̤���ѹֵ
int brk_ang = 0;      //��λ�����͵�����̤��Ƕ�
int fr_ang  = 0;      //��λ�����͵�����ǰ��ת��
int re_ang  = 0;      //��λ�����͵���������ת��
unsigned char ctrlflag = 0x00;  //��λ�����͵Ŀ����źţ������˹�/�Զ���ʻ
//��������ʵֵ
float ThrVol = 0.0;
float BrkAng = 0.0;
float FrAng  = 0.0;
float ReAng  = 0.0;
//����״̬��Ϣ
float Ax;
float Ay;
float Az;
float Yaw;
float Yaw_rate;
float Speed;
extern u16   MtSpeed;
float  VehSpeed = 0;
unsigned char receflag = 0x00; 
uint16_t adc_ConvertValue;
extern uint16_t adc_Voltage;

void ControlVehicle(void);

int main(void)
{	
//======================================Ӳ����ʼ��====================================================
	delay_init();	    	                        //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�����ж����ȼ�����2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	uart_init(115200);	//���ڳ�ʼ��Ϊ115200
	LED_Init();
	Adc_Init(DMA1_Channel1,(u32)(&(ADC1->DR)),(u32)&adc_ConvertValue,1);
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);
	Dac1_Init();
	KEY_Init();
	EXTIX_Init();
	DIR_Init();
	basic_timerInit();
	Driver_Init();
	TIM8_OPM_RCR_Init(999, 72-1);
	RS485_Init(9600);
	
	DIR_ForWard = 0; //PC11���������������IN2
	DIR_BackWard = 1;//PC12���������������IN3
	LED0 = 0;        //0 �Զ���ʻ��1 �˹���ʻ
	LED1 = 0;        //0 ǰ���� 1 ����
	uint8_t cnt = 0;
//=======================================ѭ������=====================================================
	while(1)
	{
		if (TTIM6_CheckFlag()) {
			if (cnt % 2 == 0) {
				//���Ƴ����˶�
				ControlVehicle();
				cnt = 0;
				
				
				VehSpeed = (float)MtSpeed/600 * 0.8; // rpm -> m/s ���������ٶ�
				//���͵�����������λ��
				usartSendData(VehSpeed,Ax,Ay,Yaw_rate,Yaw,receflag);
			} 
			cnt++;
		}
	} 
}

//====================================�����жϷ������=================================================
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//��������жϱ�־λ
		 //��ROS���յ�������
		 usartReceiveOneData(&thr_val,&brk_ang,&fr_ang,&re_ang,&ctrlflag);
	 }
}
//===========================================END=======================================================

//============================���Ƴ�������==========================
void ControlVehicle(void)
{
	if(LED0 == 0)
		{
			ThrVol = (float)thr_val/100;
			BrkAng = (float)brk_ang/100;
			DAC_SetChannel1Data(DAC_Align_12b_R, ThrVol*4095/3.3);
			Locate_AbsAng(BrkAng, 1500);
		}
		else
		{
			adc_Voltage = adc_ConvertValue*3300/4095 ;
			DAC_SetChannel1Data(DAC_Align_12b_R, adc_Voltage*4095/3300);
			Locate_Abs(0,1000);
		}
}
