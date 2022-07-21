#ifndef _DIR_GPIO_H
#define _DIR_GPIO_H
#include "sys.h"

#define DIR_ForWard PCout(1) //PC1控制正转
#define DIR_BackWard PCout(2) //PC2控制反转
void DIR_Init(void);


#endif

