#include "include.h"
#include "Param.h"

float g_position_pid_k = 1/100.0;


//ams1117那一架，有严重的起飞失速问题
float steady_state_roll_expect = -0.3f;
float steady_state_pitch_expect = 1.25f;
float steady_state_yaw_expect = 0.0f;

s8 gesture_feedforward_moto_0 = 20;    //前馈参数 逆时针
s8 gesture_feedforward_moto_1 = 45;
s8 gesture_feedforward_moto_2 = 21;
s8 gesture_feedforward_moto_3 = 14;

/*
//7333Q那一架，比较稳定
float steady_state_roll_expect = -0.9f;
float steady_state_pitch_expect = 0.1f;
float steady_state_yaw_expect = 0.0f;

s8 gesture_feedforward_moto_0 = 4;    //前馈参数 逆时针左上角是0
s8 gesture_feedforward_moto_1 = -6;
s8 gesture_feedforward_moto_2 = 7;
s8 gesture_feedforward_moto_3 = 9;
*/

void paramLoad(void)
{

	// The data of pitch
	ctrl.pitch.shell.kp = 5.5;
	ctrl.pitch.shell.ki = 0;
	ctrl.pitch.shell.kd = 0;
	
	ctrl.pitch.core.kp = 0.7;		
	ctrl.pitch.core.ki = 0.0005;			
	ctrl.pitch.core.kd = 2.1;			
	
	// The data of roll
	ctrl.roll.shell.kp = 5.5;		
	ctrl.roll.shell.ki = 0.0;			
	ctrl.roll.shell.kd = 0;			

	ctrl.roll.core.kp = 0.7;			
	ctrl.roll.core.ki = 0.0005;		
	ctrl.roll.core.kd = 2.1;			
	
	// The data of yaw								OK
	ctrl.yaw.shell.kp = 2;			
	ctrl.yaw.shell.ki = 0;			
	ctrl.yaw.shell.kd = 0;			
	
	ctrl.yaw.core.kp = 5;			
	ctrl.yaw.core.ki = 0.0015;			
	ctrl.yaw.core.kd = 6.0;		

	//The data of height_auto_land					
	auto_land_pid.kp = 0.5;			//
	auto_land_pid.ki = 0;			//
	auto_land_pid.kd = 4;			//
		
	//The data of set_height_cascade_control
	ctrl.height.shell.kp = 0.04f;		//		0.05				
	ctrl.height.shell.ki = 0.00000f;		//0.00005
	ctrl.height.shell.kd = 0.30f;		//
	
	ctrl.height.core.kp = 13.0f;		//	20.0	
	ctrl.height.core.ki = 0.1f;		//			
	ctrl.height.core.kd = 3.0f;		//		0.0
	
	//The data of locate_cascade_control		X
	ctrl.locate_x.shell.kp = 1.45;                  //外 1.45 ,0.04 ,0
	ctrl.locate_x.shell.ki = 0.04;
	ctrl.locate_x.shell.kd = 0.0;
	
	ctrl.locate_x.core.kp = 0.01;			          	 //内 0.01, 0.0 , 0.015
	ctrl.locate_x.core.ki = 0.0;
	ctrl.locate_x.core.kd = 0.015;
	
	//The data of locate_cascade_control		Y
	ctrl.locate_y.shell.kp = 1.45;			
	ctrl.locate_y.shell.ki = 0.04;
	ctrl.locate_y.shell.kd = 0.0;		
	
	ctrl.locate_y.core.kp = 0.01;			
	ctrl.locate_y.core.ki = 0.00;
	ctrl.locate_y.core.kd = 0.015;            

	//The data of locate_single_loop_pid_Control		X
	locate_single_loop_pid_x.kp = 0.022;       	  	//0.022
	locate_single_loop_pid_x.ki = 0.0006;							//0.0006
	locate_single_loop_pid_x.kd = 0.5;								//0.5
	
	//The data of locate_single_loop_pid_Control		Y
	locate_single_loop_pid_y.kp = 0.022;
	locate_single_loop_pid_y.ki = 0.0006;
	locate_single_loop_pid_y.kd = 0.5;
		
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 200;		//200
	ctrl.roll.shell.increment_max = 200;		//200
	ctrl.yaw.shell.increment_max = 200;			//200

	ctrl.pitch.core.increment_max = 12000;		//1000
	ctrl.roll.core.increment_max = 12000;		//1000
	ctrl.yaw.core.increment_max = 20000;		//1000	
	
	//????
	auto_land_pid.increment_max = 1000;

	//??
	ctrl.height.shell.increment_max	= 20000;
	ctrl.height.core.increment_max	= 4000;
	
	//??
	ctrl.locate_x.shell.increment_max = 200;		//200
	ctrl.locate_y.shell.increment_max = 200;		//200

	ctrl.locate_x.core.increment_max = 1000;		//1000
	ctrl.locate_y.core.increment_max = 1000;		//1000

	locate_single_loop_pid_x.increment_max = 8000;
	locate_single_loop_pid_y.increment_max = 8000;
	
	ctrl.ctrlRate = 0;
}
