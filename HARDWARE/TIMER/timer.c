#include "stm32f10x.h"
#include "timer.h"
#include "led.h"
#include "dac.h"
#include "adc.h"
#include "stepdriver.h"

uint8_t tim6_flag = 0; //中断标志位
uint16_t adc_Voltage;


void basic_timerInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
	
	//Tout = (ARR + 1) * (PSC + 1) / Tclk
	//定时器TIM6初始化, 10ms产生一次中断
	TIM_TimeBaseStructure.TIM_Period = 10000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
//	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE );

	//定时器6中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级4级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	


	TIM_Cmd(TIM6, ENABLE);  //使能TIMx	 

}
//====================================定时器6中断服务程序=================================================


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


