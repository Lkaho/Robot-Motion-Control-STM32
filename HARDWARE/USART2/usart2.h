#ifndef _USART2_H_
#define _USART2_H_
#include "stdio.h"
#include "sys.h"

#define Rece_buf_len 36 
void uart2_init(u32 bound);
void CopeSensorData(unsigned char receData);
//���ٶȽṹ�壬�ֱ���x,y,z������ٶ�
struct SAcc
{
 short A[3];
};
//���ٶȽṹ�壬�ֱ���x,y,z������ٶ�
struct SAngu_Vol                                          
{
 short Angu[3];
};
//�ǶȽṹ�壬�ֱ���x,y,z����Ƕ�
struct SAngle
{
 short Ang[3];
};


#endif
