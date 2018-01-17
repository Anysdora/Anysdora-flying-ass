#ifndef __TIME_H__
#define __TIME_H__
#include "stm32f10x.h"

#define EnTIM3()  TIM_Cmd(TIM3,ENABLE)
void TIM3_Init(u16 period_num);


void TIM1_Init(void);



//此函数用于DMP6050的DMP的时间戳，然而并没有什么卵用
void  get_ms_gansha(unsigned long *count);

#endif 
