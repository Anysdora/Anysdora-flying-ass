#ifndef _PARAM_H_
#define _PARAM_H_

#include "include.h"


extern float g_position_pid_k;

extern s8 gesture_feedforward_moto_0;
extern s8 gesture_feedforward_moto_1;
extern s8 gesture_feedforward_moto_2;
extern s8 gesture_feedforward_moto_3;

extern float steady_state_roll_expect;
extern float steady_state_pitch_expect;
extern float steady_state_yaw_expect;

void paramLoad(void);

#endif /* __EEPROM_H */


