#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11

//��linux���ղ��������ݵ�������ַ��
extern int usartReceiveOneData(int *p_thrSet,int *p_brkSet, int *p_FrontAngSet, int *p_RearAngSet,unsigned char *p_crtlFlag);
//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux
void usartSendData(float Speed_ac, float Accx_ac, float Accy_ac, float Yaw_rateac, float Yaw_ac, unsigned char ctrlFlag);
//����ָ���ַ�����ĺ���
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 

#endif