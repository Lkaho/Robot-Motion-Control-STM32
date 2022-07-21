#ifndef _USART2_H_
#define _USART2_H_
#include "stdio.h"
#include "sys.h"

#define Rece_buf_len 36 
void uart2_init(u32 bound);
void CopeSensorData(unsigned char receData);
//加速度结构体，分别存放x,y,z方向加速度
struct SAcc
{
 short A[3];
};
//角速度结构体，分别存放x,y,z方向角速度
struct SAngu_Vol                                          
{
 short Angu[3];
};
//角度结构体，分别存放x,y,z方向角度
struct SAngle
{
 short Ang[3];
};


#endif
