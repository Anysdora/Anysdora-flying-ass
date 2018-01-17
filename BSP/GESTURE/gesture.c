#include "gesture.h"
#include "include.h"

u8 ARMED = 0;

vs16 Moto_duty[4];//typedef __IO int16_t  vs16;

struct _ctrl ctrl;
/********************************************************/
/*void X_Control(float rol, float pit, float yaw)		*/
/*输入：rol   横滚角                               		*/
/*      pit   俯仰角                                 	*/
/*		yaw   航向角                                 	*/
/*输出：                                             	*/
/********************************************************/

//float steady_state_roll_expect = 0.0f;
//float steady_state_pitch_expect = 0.0f;
//float steady_state_yaw_expect = 0.0f;

void X_CONTROL(float rol, float pit, float yaw)
{		
	float roll_error, pitch_error, yaw_error;
	static float roll_error_last, pitch_error_last, yaw_error_last;
	
	if(ctrl.ctrlRate >= 2) 													
	{
		//*****************外环PID**************************//
		
		//*********************************************俯仰**********************************************//
		
		
		//pitch_offset 定点输出
		pitch_error = pitch_offset - pit + steady_state_pitch_expect;	//俯仰offset-当前pitch+角度修正
	

		ctrl.pitch.shell.increment += pitch_error;   			//pitch外环误差累积
		//积分限幅
		if(ctrl.pitch.shell.increment > ctrl.pitch.shell.increment_max)
				ctrl.pitch.shell.increment = ctrl.pitch.shell.increment_max;
		else if(ctrl.pitch.shell.increment < -ctrl.pitch.shell.increment_max)
				ctrl.pitch.shell.increment = -ctrl.pitch.shell.increment_max;
		
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * pitch_error      //pid算法公式
								 + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment 
								 + ctrl.pitch.shell.kd * (pitch_error - pitch_error_last);
	
		pitch_error_last = pitch_error; 
		
		//*********************************************横滚**********************************************//
		
				//roll_offset 定点输出
				roll_error = roll_offset - rol + steady_state_roll_expect;


		ctrl.roll.shell.increment += roll_error;  
		//积分限幅
		if(ctrl.roll.shell.increment > ctrl.roll.shell.increment_max)
				ctrl.roll.shell.increment = ctrl.roll.shell.increment_max;
		else if(ctrl.roll.shell.increment < -ctrl.roll.shell.increment_max)
				ctrl.roll.shell.increment = -ctrl.roll.shell.increment_max;

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * roll_error 
								 + ctrl.roll.shell.ki * ctrl.roll.shell.increment 
								 + ctrl.roll.shell.kd * (roll_error - roll_error_last);
	
		roll_error_last = roll_error;  

		//*********************************************偏航**********************************************//
		yaw_error = yaw_offset - yaw;			// + (Rc_Data.YAW - Rc_Data.yaw_offset)/5
		
		//对 angle.yaw 在180°附近做连续处理
		if(yaw_error > 180)
		{
			yaw_error = yaw_error - 360;
		}
		if(yaw_error < -180)
		{
			yaw_error = 360 + yaw_error;
		}
		

		ctrl.yaw.shell.increment += yaw_error;  		
		//积分限幅
		if(ctrl.yaw.shell.increment > ctrl.yaw.shell.increment_max)
				ctrl.yaw.shell.increment = ctrl.yaw.shell.increment_max;
		else if(ctrl.yaw.shell.increment < -ctrl.yaw.shell.increment_max)
				ctrl.yaw.shell.increment = -ctrl.yaw.shell.increment_max;

		ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * yaw_error
							   + ctrl.yaw.shell.ki * ctrl.yaw.shell.increment
							   + ctrl.yaw.shell.kd * (yaw_error - yaw_error_last);		
		
		yaw_error_last = yaw_error;
//注意：在这里使用yaw_error值的差、yaw值的差和sensor.gyro.radian.z的值是有区别的，
//yaw_error值的差、yaw值的差在期望值改变时不相等,yaw值的差和sensor.gyro.radian.z的值不相等，因为频率不同
		
		ctrl.ctrlRate = 0;//清零 也就是说按照内环先执行两次，外环执行一次
	}
	
	ctrl.ctrlRate ++;
	
	//*******************************************内环(角速度环)PID*********************************//	

	//********************************************横滚**********************************************//
	float roll_rate_error, pitch_rate_error, yaw_rate_error;
	
	//微分先行, 内环的期望值一直在改变, 不使用微分先行会导致内环输出剧烈波动
	static float roll_rate_last, pitch_rate_last, yaw_rate_last;
	
	roll_rate_error = ctrl.roll.shell.pid_out - sensor.gyro.angle.y;  //sensor.gyro.angle.y为陀螺仪得出的角速度
	ctrl.roll.core.kp_out = ctrl.roll.core.kp * roll_rate_error;
	
	ctrl.roll.core.increment += roll_rate_error;   	
	if(ctrl.roll.core.increment > ctrl.roll.core.increment_max)
			ctrl.roll.core.increment = ctrl.roll.core.increment_max;
	else if(ctrl.roll.core.increment < -ctrl.roll.core.increment_max)
			ctrl.roll.core.increment = -ctrl.roll.core.increment_max;
	ctrl.roll.core.ki_out = ctrl.roll.core.ki * ctrl.roll.core.increment; 
	
	ctrl.roll.core.kd_out = ctrl.roll.core.kd * (roll_rate_last - sensor.gyro.angle.y);
	roll_rate_last = sensor.gyro.angle.y;

	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.ki_out + ctrl.roll.core.kd_out;//作为最后在roll的输出pid

	
	//*********************************************俯仰**********************************************//
	pitch_rate_error = ctrl.pitch.shell.pid_out - sensor.gyro.angle.x;
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * pitch_rate_error;

	ctrl.pitch.core.increment += pitch_rate_error;   		
	if(ctrl.pitch.core.increment > ctrl.pitch.core.increment_max)
			ctrl.pitch.core.increment = ctrl.pitch.core.increment_max;
	else if(ctrl.pitch.core.increment < -ctrl.pitch.core.increment_max)
			ctrl.pitch.core.increment = -ctrl.pitch.core.increment_max;
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment; 
	
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (pitch_rate_last - sensor.gyro.angle.x);	
	pitch_rate_last = sensor.gyro.angle.x;

	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;

	//*********************************************偏航**********************************************//
	yaw_rate_error = ctrl.yaw.shell.pid_out - sensor.gyro.angle.z;
	
	ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * yaw_rate_error;
	
	ctrl.yaw.core.increment += yaw_rate_error;   		
	if(ctrl.yaw.core.increment > ctrl.yaw.core.increment_max)
			ctrl.yaw.core.increment = ctrl.yaw.core.increment_max;
	else if(ctrl.yaw.core.increment < -ctrl.yaw.core.increment_max)
			ctrl.yaw.core.increment = -ctrl.yaw.core.increment_max;
	ctrl.yaw.core.ki_out = ctrl.yaw.core.ki * ctrl.yaw.core.increment; 

	ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (yaw_rate_last - sensor.gyro.angle.z);
	yaw_rate_last = sensor.gyro.angle.z;
	
	ctrl.yaw.core.pid_out =  ctrl.yaw.core.kp_out + ctrl.yaw.core.ki_out + ctrl.yaw.core.kd_out;
}


