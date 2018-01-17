#ifndef __TIM_PWM_IN_H_
#define __TIM_PWM_IN_H_
#include "stm32f10x.h"

typedef struct int16_rcget
{
	int16_t ROLL;
	int16_t PITCH;
	int16_t THROTTLE;
	int16_t YAW;
	int16_t pitch_offset;
	int16_t roll_offset;
	int16_t yaw_offset;
}T_RC_DATA;
	
extern T_RC_DATA Rc_Data;//1000~2000

//后增加的四路通道
extern u16 Rc_Data_T8_C1;	
extern u16 Rc_Data_T8_C2;
extern u16 Rc_Data_T8_C3;  //0--1087, 1--1487, 2--1887 上下浮动在1、2之间
extern u16 Rc_Data_T8_C4;

void TIM4_Cap_Init(void);
void GET_FOUR_PWM(void);
void GET_TIM8_PWM(void);
#endif
