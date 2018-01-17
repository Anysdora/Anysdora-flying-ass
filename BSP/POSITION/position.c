#include "include.h"
#include "position.h"


//
float pitch_offset = 0;				//
float roll_offset = 0;				//
float yaw_offset = 0;
float yaw_offset_tmp = 0;

enum _position_mode position_mode;

//
struct _position_float set_point_expect_position = {0 , 0};	//struct _position_float
																														//{
																														//			float x;
																														//			float y;
																														//};
//
//{-15, -9}--45cm?--?????????????



void locate_cascade_control(float coordinate_x, float coordinate_y)
{
	float x_error, y_error;    //坐标误差
	static float x_error_last, y_error_last;//上一次坐标误差

	float x_speed_error, y_speed_error;//速度误差
	static float x_speed_error_last, y_speed_error_last;//上一次速度误差
	static float coordinate_x_last, coordinate_y_last;//上一次坐标
	

		//*****************外环**************************//
		
		//********************** X **********************//
		x_error = set_point_expect_position.x - coordinate_x;

		ctrl.locate_x.shell.increment += x_error; //积分累加  			
		//积分限幅
		if(ctrl.locate_x.shell.increment > ctrl.locate_x.shell.increment_max)
				ctrl.locate_x.shell.increment = ctrl.locate_x.shell.increment_max;
		else if(ctrl.locate_x.shell.increment < -ctrl.locate_x.shell.increment_max)
				ctrl.locate_x.shell.increment = -ctrl.locate_x.shell.increment_max;
		
		ctrl.locate_x.shell.pid_out = ctrl.locate_x.shell.kp * x_error 
									+ ctrl.locate_x.shell.ki * ctrl.locate_x.shell.increment 
									+ ctrl.locate_x.shell.kd * (x_error - x_error_last);

		x_error_last = x_error; 
		
		//********************** Y  ***********************//
		y_error = set_point_expect_position.y - coordinate_y;

		ctrl.locate_y.shell.increment += y_error;  
		
		if(ctrl.locate_y.shell.increment > ctrl.locate_y.shell.increment_max)
				ctrl.locate_y.shell.increment = ctrl.locate_y.shell.increment_max;
		else if(ctrl.locate_y.shell.increment < -ctrl.locate_y.shell.increment_max)
				ctrl.locate_y.shell.increment = -ctrl.locate_y.shell.increment_max;

		ctrl.locate_y.shell.pid_out  = ctrl.locate_y.shell.kp * y_error 
									 + ctrl.locate_y.shell.ki * ctrl.locate_y.shell.increment 
									 + ctrl.locate_y.shell.kd * (y_error - y_error_last);
	
		y_error_last = y_error;  
		
		
	
	
	
	
	//********************内环*********************************//	
	


	//********************** X  ***********************//
	
		
	x_speed_error = ctrl.locate_x.shell.pid_out    //往前V为
				 + (coordinate_x - coordinate_x_last) / 0.05;  //	cm/s
	
	coordinate_x_last = coordinate_x;
	
	ctrl.locate_x.core.kp_out = ctrl.locate_x.core.kp * x_speed_error;
	
	ctrl.locate_x.core.increment += x_speed_error;  //限幅 	
	if(ctrl.locate_x.core.increment > ctrl.locate_x.core.increment_max)
			ctrl.locate_x.core.increment = ctrl.locate_x.core.increment_max;
	else if(ctrl.locate_x.core.increment < -ctrl.locate_x.core.increment_max)
			ctrl.locate_x.core.increment = -ctrl.locate_x.core.increment_max;
	ctrl.locate_x.core.ki_out = ctrl.locate_x.core.ki * ctrl.locate_x.core.increment; 
	
	ctrl.locate_x.core.kd_out = ctrl.locate_x.core.kd * (x_speed_error - x_speed_error_last);
	
	x_speed_error_last = x_speed_error;

	ctrl.locate_x.core.pid_out = ctrl.locate_x.core.kp_out 
							   + ctrl.locate_x.core.ki_out 
							   + ctrl.locate_x.core.kd_out;
	
	//********************** Y ***********************//
	y_speed_error = ctrl.locate_y.shell.pid_out
				 + (coordinate_y - coordinate_y_last) / 0.05;					//	cm/s
	
	coordinate_y_last = coordinate_y;
	
	ctrl.locate_y.core.kp_out = ctrl.locate_y.core.kp * y_speed_error;

	ctrl.locate_y.core.increment += y_speed_error;   		
	if(ctrl.locate_y.core.increment > ctrl.locate_y.core.increment_max)
			ctrl.locate_y.core.increment = ctrl.locate_y.core.increment_max;
	else if(ctrl.locate_y.core.increment < -ctrl.locate_y.core.increment_max)
			ctrl.locate_y.core.increment = -ctrl.locate_y.core.increment_max;
	ctrl.locate_y.core.ki_out = ctrl.locate_y.core.ki * ctrl.locate_y.core.increment; 
	
	ctrl.locate_y.core.kd_out = ctrl.locate_y.core.kd * (y_speed_error - y_speed_error_last);
	
	y_speed_error_last = y_speed_error;

	ctrl.locate_y.core.pid_out = ctrl.locate_y.core.kp_out 
							   + ctrl.locate_y.core.ki_out 
							   + ctrl.locate_y.core.kd_out;
	
	pitch_offset = ctrl.locate_y.core.pid_out;
	roll_offset = ctrl.locate_x.core.pid_out;
	
	if(pitch_offset >  5)	pitch_offset =  5;
	if(pitch_offset < -5)	pitch_offset = -5;
	if(roll_offset  >  5)	roll_offset  =  5;
	if(roll_offset  < -5)	roll_offset  = -5;
}


//??:???pid
struct _pid locate_single_loop_pid_x;
struct _pid locate_single_loop_pid_y;

void locate_single_loop_pid_control(float coordinate_x, float coordinate_y)   //定点PID
{
	float x_error, y_error;    //坐标误差
	static float x_error_last, y_error_last;//上一次坐标误差

	float x_speed_error, y_speed_error;//速度误差
	static float x_speed_error_last, y_speed_error_last;//上一次速度误差
	static float coordinate_x_last, coordinate_y_last;//上一次坐标
	
//	if(g_height>50)
//	{
//		if(fabs(coordinate_x-coordinate_x_last)>80)
//		{
//			coordinate_x = coordinate_x_last;
//		}
//		
//		if(fabs(coordinate_y-coordinate_y_last)>80)
//		{
//			coordinate_y = coordinate_y_last;
//		}
//			
//	}

		//*****************外环**************************//
		
		//********************** X **********************//
		x_error = set_point_expect_position.x - coordinate_x;

		ctrl.locate_x.shell.increment += x_error; //积分累加  			
		//积分限幅
		if(ctrl.locate_x.shell.increment > ctrl.locate_x.shell.increment_max)
				ctrl.locate_x.shell.increment = ctrl.locate_x.shell.increment_max;
		else if(ctrl.locate_x.shell.increment < -ctrl.locate_x.shell.increment_max)
				ctrl.locate_x.shell.increment = -ctrl.locate_x.shell.increment_max;
		
		ctrl.locate_x.shell.pid_out = ctrl.locate_x.shell.kp * x_error 
									+ ctrl.locate_x.shell.ki * ctrl.locate_x.shell.increment 
									+ ctrl.locate_x.shell.kd * (x_error - x_error_last);

		x_error_last = x_error; 
		
		//********************** Y  ***********************//
		y_error = set_point_expect_position.y - coordinate_y;

		ctrl.locate_y.shell.increment += y_error;  
		
		if(ctrl.locate_y.shell.increment > ctrl.locate_y.shell.increment_max)
				ctrl.locate_y.shell.increment = ctrl.locate_y.shell.increment_max;
		else if(ctrl.locate_y.shell.increment < -ctrl.locate_y.shell.increment_max)
				ctrl.locate_y.shell.increment = -ctrl.locate_y.shell.increment_max;

		ctrl.locate_y.shell.pid_out  = ctrl.locate_y.shell.kp * y_error 
									 + ctrl.locate_y.shell.ki * ctrl.locate_y.shell.increment 
									 + ctrl.locate_y.shell.kd * (y_error - y_error_last);
	
		y_error_last = y_error;  
		
		
	
	
	
	
	//********************内环*********************************//	
	


	//********************** X  ***********************//
	
		
	x_speed_error = ctrl.locate_x.shell.pid_out    //往前飞x减小
				 - (coordinate_x - coordinate_x_last) / 0.05;  //	cm/s
	
	coordinate_x_last = coordinate_x;
	
	ctrl.locate_x.core.kp_out = ctrl.locate_x.core.kp * x_speed_error;
	
	ctrl.locate_x.core.increment += x_speed_error;  //限幅 	
	if(ctrl.locate_x.core.increment > ctrl.locate_x.core.increment_max)
			ctrl.locate_x.core.increment = ctrl.locate_x.core.increment_max;
	else if(ctrl.locate_x.core.increment < -ctrl.locate_x.core.increment_max)
			ctrl.locate_x.core.increment = -ctrl.locate_x.core.increment_max;
	ctrl.locate_x.core.ki_out = ctrl.locate_x.core.ki * ctrl.locate_x.core.increment; 
	
	ctrl.locate_x.core.kd_out = ctrl.locate_x.core.kd * (x_speed_error - x_speed_error_last);
	
	x_speed_error_last = x_speed_error;

	ctrl.locate_x.core.pid_out = ctrl.locate_x.core.kp_out 
							   + ctrl.locate_x.core.ki_out 
							   + ctrl.locate_x.core.kd_out;
	
	//********************** Y ***********************//
	y_speed_error = ctrl.locate_y.shell.pid_out
				 - (coordinate_y - coordinate_y_last) / 0.05;					//	cm/s
	
	coordinate_y_last = coordinate_y;
	
	ctrl.locate_y.core.kp_out = ctrl.locate_y.core.kp * y_speed_error;

	ctrl.locate_y.core.increment += y_speed_error;   		
	if(ctrl.locate_y.core.increment > ctrl.locate_y.core.increment_max)
			ctrl.locate_y.core.increment = ctrl.locate_y.core.increment_max;
	else if(ctrl.locate_y.core.increment < -ctrl.locate_y.core.increment_max)
			ctrl.locate_y.core.increment = -ctrl.locate_y.core.increment_max;
	ctrl.locate_y.core.ki_out = ctrl.locate_y.core.ki * ctrl.locate_y.core.increment; 
	
	ctrl.locate_y.core.kd_out = ctrl.locate_y.core.kd * (y_speed_error - y_speed_error_last);
	
	y_speed_error_last = y_speed_error;

	ctrl.locate_y.core.pid_out = ctrl.locate_y.core.kp_out 
							   + ctrl.locate_y.core.ki_out 
							   + ctrl.locate_y.core.kd_out;
	
	roll_offset = ctrl.locate_y.core.pid_out;
	pitch_offset = ctrl.locate_x.core.pid_out;
	
	if(pitch_offset >  5)	pitch_offset =  5;
	if(pitch_offset < -5)	pitch_offset = -5;
	if(roll_offset  >  5)	roll_offset  =  5;
	if(roll_offset  < -5)	roll_offset  = -5;
}

//
void track_output_limit(float max, float *pitch)
{
	if(*pitch >  max)	*pitch =  max;
	if(*pitch < -max)	*pitch = -max;	
}

//
//
//@brief :	ncurrent_position
//
//@param :	1.
//			2.
//		 	3.
//		 	4.
//
//@retval:	1--
//			0--
//
//@note	 :
//
u8 stady_state_judge(struct _position_float current_position, struct _position_float expect_position, u16 n_times, u16 steady_threshold)
{
	static u16 continues_times = 0;

	if(current_position.x - expect_position.x < steady_threshold && current_position.x - expect_position.x > -steady_threshold &&
	   current_position.y - expect_position.y < steady_threshold && current_position.y - expect_position.y > -steady_threshold)
	{
		continues_times++;
	}	
	else
	{
		continues_times  = 0;
	}
		
	if(continues_times >= n_times)
	{
		//??
		continues_times  = 0;

		//???1?, ?????????????????
		return 1;
	}
	else
	{
		return 0;
	}
	
}




//????????????--???????????
void position_mode_handle(void)
{
	if(position_mode == libertine)
	{
		roll_offset = 0;
		pitch_offset = 0;
		
	}
	else if(position_mode == locate)
	{
//		locate_cascade_control(0, 0);
		
		//?? > 40cm?,?????
		if(g_height > 40)
		{
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
		}
		else
		{
			roll_offset = 0;
			pitch_offset = 0;
		}
	}
	else if(position_mode == track)
	{
		roll_offset = 0;
		pitch_offset = 0;

	}
	else
	{
		//do nothing
	}
}
