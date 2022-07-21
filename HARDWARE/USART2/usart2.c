#include "usart2.h"
#include <string.h>
//u8 DMA_Rece_Buf[Rece_buf_len]= {0};
u32 len;

float Ax;
float Ay;
float Az;
float Yaw;
float Yaw_rate;
struct SAcc Acc;
struct SAngu_Vol Angular_V;
struct SAngle Angle;

unsigned char ReceBuffer[11]; //串口接受传感器数据缓存区


void uart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART2时钟


	//USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2

	//USART2_RX	  GPIOA.3初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口2接收中断

	
	USART_Cmd(USART2, ENABLE);                    //使能串口2
}
void CopeSensorData(unsigned char receData)
{
	static unsigned char RxCout = 0; //索引
	
	ReceBuffer[RxCout++] = receData;//接从串口接收一个字节，RxCout自增1
	if(ReceBuffer[0] != 0x55) //如果没有接收到帧头，舍弃这个数据
	{
		RxCout = 0;
	  return;
	}
	if(RxCout<11)//接收一组数据的长度不满足则继续接收
	{ return;
	}
	else
	{
		switch(ReceBuffer[1])
		{
			case 0x51: 
				memcpy(&Acc, &ReceBuffer[2],6);
				Ax = (float)Acc.A[0]/32768*16;
		    Ay = (float)Acc.A[1]/32768*16;
			  Az = (float)Acc.A[2]/32768*16;
			  break;
			case 0x52:
				memcpy(&Angular_V, &ReceBuffer[2],6);
				Yaw_rate = (float)Angular_V.Angu[2]/32768.0*2000;
			  break;
			case 0x53:
				memcpy(&Angle, &ReceBuffer[2],6);
				Yaw = (float)Angle.Ang[2]/32768.0*180;
			  break;		  
		}
		RxCout = 0;//清空接收区
	}
}

//====================================串口2中断服务程序=================================================
//对加速度传感器接收到的数据进行解析成真实数据
void USART2_IRQHandler(void) 
{

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //
	{
    CopeSensorData((unsigned char)USART2->DR);//处理数据
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}

}

