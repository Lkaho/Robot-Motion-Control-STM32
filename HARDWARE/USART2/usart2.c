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

unsigned char ReceBuffer[11]; //���ڽ��ܴ��������ݻ�����


void uart2_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //ʹ��USART2ʱ��


	//USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2

	//USART2_RX	  GPIOA.3��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3  

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������2�����ж�

	
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2
}
void CopeSensorData(unsigned char receData)
{
	static unsigned char RxCout = 0; //����
	
	ReceBuffer[RxCout++] = receData;//�ӴӴ��ڽ���һ���ֽڣ�RxCout����1
	if(ReceBuffer[0] != 0x55) //���û�н��յ�֡ͷ�������������
	{
		RxCout = 0;
	  return;
	}
	if(RxCout<11)//����һ�����ݵĳ��Ȳ��������������
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
		RxCout = 0;//��ս�����
	}
}

//====================================����2�жϷ������=================================================
//�Լ��ٶȴ��������յ������ݽ��н�������ʵ����
void USART2_IRQHandler(void) 
{

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //
	{
    CopeSensorData((unsigned char)USART2->DR);//��������
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}

}

