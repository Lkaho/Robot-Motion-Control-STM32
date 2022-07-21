#include "stm32f10x.h"
#include "timer.h"
#include "led.h"
#include "dac.h"
#include "adc.h"
#include "stepdriver.h"

uint8_t tim6_flag = 0; //�жϱ�־λ
uint16_t adc_Voltage;


void basic_timerInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʱ��ʹ��
	
	//Tout = (ARR + 1) * (PSC + 1) / Tclk
	//��ʱ��TIM6��ʼ��, 10ms����һ���ж�
	TIM_TimeBaseStructure.TIM_Period = 10000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
//	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE );

	//��ʱ��6�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�4��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	


	TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx	 

}
//====================================��ʱ��6�жϷ������=================================================


void TIM6_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM6 , TIM_IT_Update)!=RESET)
	{
	  tim6_flag = 1;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}

uint8_t TIM6_CheckFlag(void)
{
	if (tim6_flag == 1) {
		tim6_flag = 0;
		return 1;
	} else {
		return 0;
	}
}


