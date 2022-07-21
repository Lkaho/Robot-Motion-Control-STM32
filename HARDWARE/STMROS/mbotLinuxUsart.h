#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include <sys.h>	

#define START   0X11

//从linux接收并解析数据到参数地址中
extern int usartReceiveOneData(int *p_thrSet,int *p_brkSet, int *p_FrontAngSet, int *p_RearAngSet,unsigned char *p_crtlFlag);
//封装数据，调用USART1_Send_String将数据发送给linux
void usartSendData(float Speed_ac, float Accx_ac, float Accy_ac, float Yaw_rateac, float Yaw_ac, unsigned char ctrlFlag);
//发送指定字符数组的函数
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 

#endif
