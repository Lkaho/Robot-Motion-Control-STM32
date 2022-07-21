#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "stm32f10x.h"
void basic_timerInit(void);
void Speed_CouterInit(void);
uint8_t TTIM6_CheckFlag(void);
#endif /* _TIMER_H */

