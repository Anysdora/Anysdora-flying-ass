#include "include.h"
#include "CONTROL.h"

//电机输出处理
void MOTO_output_handle(void)
{	
	
		{
			if(mode_machine == mode_none)//模式0 电机不转
			{
				Moto_duty[0] = 0;
				Moto_duty[1] = 0;
				Moto_duty[2] = 0;
				Moto_duty[3] = 0;
			}
			else if(mode_machine == B_to_A || mode_machine == FIND_LCUK || mode_machine == A_C_B || mode_machine == fly_on_rectangle || mode_machine == set_height_ || mode_machine == set_point_turning||mode_machine ==A_to_B||mode_machine ==A_to_B_fire_B_to_A)
			{
				Moto_duty[0] = set_height_throttle_grab.current_throttle  - 1000 + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[1] = set_height_throttle_grab.current_throttle  - 1000 + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
				Moto_duty[2] = set_height_throttle_grab.current_throttle  - 1000 - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[3] = set_height_throttle_grab.current_throttle  - 1000 - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
				
				//set_height_throttle_grab.current_throttle（抓取油门）  - 1000
				
				Moto_duty[0] += ctrl.height.core.pid_out + auto_land_pid.pid_out;
				Moto_duty[1] += ctrl.height.core.pid_out + auto_land_pid.pid_out;
				Moto_duty[2] += ctrl.height.core.pid_out + auto_land_pid.pid_out;
				Moto_duty[3] += ctrl.height.core.pid_out + auto_land_pid.pid_out;
			}
			else if(mode_machine == gesture_turning)   //调PID姿态的模式gesture_turning
			{
				Moto_duty[0] = 300 + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out;// + ctrl.yaw.core.pid_out;
				Moto_duty[1] = 300 + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out;// - ctrl.yaw.core.pid_out;
				Moto_duty[2] = 300 - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out;// + ctrl.yaw.core.pid_out;
				Moto_duty[3] = 300 - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out;// - ctrl.yaw.core.pid_out;
			}
			else
			{
				Moto_duty[0] = 0;
				Moto_duty[1] = 0;
				Moto_duty[2] = 0;
				Moto_duty[3] = 0;
			}
				
			if(Moto_duty[0]<=0) Moto_duty[0] = 0;
			if(Moto_duty[1]<=0) Moto_duty[1] = 0;
			if(Moto_duty[2]<=0) Moto_duty[2] = 0;
			if(Moto_duty[3]<=0) Moto_duty[3] = 0;
		}

			
	if(ARMED)  //如果解锁
	{				
//		Moto_duty[0] = 0;
//		Moto_duty[1] = 0;
//		Moto_duty[2] = 0;
//		Moto_duty[3] = 0;
		mode_machine = set_height_;
//		mode_machine = gesture_turning;
//		mode_machine = set_point_turning;
//		mode_machine = A_to_B;
//		mode_machine =A_to_B_fire_B_to_A;
//		mode_machine =fly_on_rectangle;
//		mode_machine = B_to_A;
		
		
		Moto_PwmRflash(Moto_duty[0], Moto_duty[1], Moto_duty[2], Moto_duty[3]);

	}
	else
		Moto_PwmRflash(0,0,0,0);		
}



		

//四轴失控pitch, roll, yaw超过一定角度时，自动上锁
//防止倾角太大，出现意外事故
void out_of_control_lock(const struct _angle *current_angle)
{
	if(																	//(current_angle->yaw > 90)  || (current_angle->yaw < -90)  ||
	   (current_angle->pitch > 12) || (current_angle->pitch < -12) ||
	   (current_angle->roll > 12)  || (current_angle->roll < -12))
	{
		//四轴解除武装
		ARMED = 0;
	}
}

