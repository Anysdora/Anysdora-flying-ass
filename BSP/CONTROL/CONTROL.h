#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f10x.h"
#include "_10_axis.h"
#include "include.h"


//电机输出处理
void MOTO_output_handle(void);
	

void Deblocking(void);

//NRF24L01 上锁解锁
void arm_disarm(void);

//四轴失控pitch, roll, yaw超过一定角度时，自动上锁
//防止倾角太大，出现意外事故
void out_of_control_lock(const struct _angle *current_angle);

#endif
