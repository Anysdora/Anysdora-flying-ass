#ifndef _MOTO_H_
#define _MOTO_H_
#include "stm32f10x.h"

#define Moto_PwmMax 950


extern s8 gesture_feedforward_moto_0;
extern s8 gesture_feedforward_moto_1;
extern s8 gesture_feedforward_moto_2;
extern s8 gesture_feedforward_moto_3;


void Moto_PwmRflash(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM);
void Moto_Init(void);

#endif
