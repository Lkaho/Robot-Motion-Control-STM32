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
//声明的变量

//底盘控制变量(放大了100倍)
int thr_val = 0;      //上位机发送的期望踏板电压值
int brk_ang = 0;      //上位机发送的期望踏板角度
int fr_ang  = 0;      //上位机发送的期望前轮转角
int re_ang  = 0;      //上位机发送的期望后轮转角
unsigned char ctrlflag = 0x00;  //上位机发送的控制信号，控制人工/自动驾驶
//控制量真实值
float ThrVol = 0.0;
float BrkAng = 0.0;
float FrAng  = 0.0;
float ReAng  = 0.0;
//车辆状态信息
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
//======================================硬件初始化====================================================
	delay_init();	    	                        //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	uart_init(115200);	//串口初始化为115200
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
	
	DIR_ForWard = 0; //PC11接驱动电机控制器IN2
	DIR_BackWard = 1;//PC12接驱动电机控制器IN3
	LED0 = 0;        //0 自动驾驶；1 人工驾驶
	LED1 = 0;        //0 前进； 1 倒车
	uint8_t cnt = 0;
//=======================================循环程序=====================================================
	while(1)
	{
		if (TTIM6_CheckFlag()) {
			if (cnt % 2 == 0) {
				//控制车辆运动
				ControlVehicle();
				cnt = 0;
				
				
				VehSpeed = (float)MtSpeed/600 * 0.8; // rpm -> m/s 后轮中心速度
				//发送底盘数据至上位机
				usartSendData(VehSpeed,Ax,Ay,Yaw_rate,Yaw,receflag);
			} 
			cnt++;
		}
	} 
}

//====================================串口中断服务程序=================================================
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//首先清除中断标志位
		 //从ROS接收到的数据
		 usartReceiveOneData(&thr_val,&brk_ang,&fr_ang,&re_ang,&ctrlflag);
	 }
}
//===========================================END=======================================================

//============================控制车辆程序==========================
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
