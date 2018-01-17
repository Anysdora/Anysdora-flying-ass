#ifndef _GESTURE_H_
#define _GESTURE_H_

#include "include.h"


struct _pid
{
	float kp;
	float ki;
	float kd;
	float increment;
	float increment_max;
	float kp_out;
	float ki_out;
	float kd_out;
	float pid_out;
};

struct _tache
{
	struct _pid shell;
	struct _pid core;	
};
	
struct _ctrl
{
	u8  ctrlRate;
	struct _tache pitch;    
	struct _tache roll;  
	struct _tache yaw;   
	struct _tache height;
	struct _tache locate_x;
	struct _tache locate_y;
};

extern struct _ctrl ctrl;						
						
extern u8 ARMED;
extern vs16 Moto_duty[4];

void X_CONTROL(float rol, float pit, float yaw);


#endif
