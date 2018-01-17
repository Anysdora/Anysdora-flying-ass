#include "include.h"
#include "business_mode.h"


//@brief :	系统第n次调用wait_for_n_ms函数, 
//			该变量 必须+必须+必须(重要的事情要说三遍) 
//			在 ①"update_last_mode_state()函数(函数外)" 中以及 ②"每次等待时间到达后"(函数内) 更新
//
u16 x_times_call_wait_for_n_ms_func = 0;

u8 Flag=0;
//
//@brief :	等待 n * 10(ms), 主要用于流程控制中的时间等待
//
//@param :	param_0 system_time_tick, 系统时钟滴, 频率100HZ
//			param_1 n_ms,
//			param_2 此处是第几次调用该函数
//
//@retval:	等待时间是否已到，到则 return 1, 否则 return 0.
//
//@note  :	param_2 主要是用来清零时间，重新计时， 每次等待时间到达后必须更新
//			此函数不能交叉调用，即在 n_ms 计时未到时，不能在其他处被调用，否则会出错
//
u8 wait_for_n_ms(u32 *system_time_tick, u32 n_ms, u16 *x_times)
{
	//初始化为65535, 防止第一次调用时 x_times_last == x_times, 从而产生bug
	static u16 x_times_last = 65535;

	//当再次调用到此函数时，记录下调用瞬间的system_time_tick, 用来与 n_ms 做比较, 看是否已到达等待时间。
	
	//开始等待时将 "system_time_tick" 锁存
	static u32 start_wait_system_time_tick_latch;

	if(*x_times == x_times_last) 
	{		
		if(n_ms < (*system_time_tick - start_wait_system_time_tick_latch))
		{
			//时间已到, 必须更新变量--->x_times_call_wait_for_n_ms_func
			(*x_times)++; 
	
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		x_times_last = *x_times;
		
		//时间锁存
		start_wait_system_time_tick_latch = *system_time_tick;
				
		return 0;
	}
}




//

int16_t MODE=0;
//
void A_to_B_fire_B_to_A_control(mode_flow_step_type *current_mode_flow_steps)
{
	u8 Suddenly_flag = 1;
  
	float track_pitch_offset_max = 2.3f;
	
	if(g_height == 300) g_height =110;
	
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			//让偏航角的期望值等于起飞瞬间的yaw值, 这样就不会导致上电后四轴被特意旋转后起飞导致失控
			yaw_offset = angle.yaw;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
			set_height_throttle_grab.current_throttle = 0 ;
			fly_direction = forward;
			magnet_ON;
		
			break;
		}
		
		case mode_step_1:
		{
			MODE=1;
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:
		{
			MODE=2;
//			set_height_control_expect_height = delivery_param_to_control.expect_height_start_circle;
			set_height_control_expect_height =110;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
	
			if(g_height > 35)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
				if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 5, 80))
				{
					*current_mode_flow_steps = mode_step_3;
					
					//取yaw期望值
//					yaw_offset_tmp = get_yaw_expect(g_position_circle_adjust.x, g_position_circle_adjust.y, 
//													g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				}
				

			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_3:
		{
			MODE=3;
//			if(yaw_expect_wave(yaw_offset_tmp, &yaw_offset))
			{
				*current_mode_flow_steps = mode_step_4;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 35)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_4:
		{
			MODE=4;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 35)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			//进入循迹步
//			if(g_height > set_height_control_expect_height - 5 && g_height < set_height_control_expect_height + 5)
//			{
//				*current_mode_flow_steps = mode_step_5;
//			}
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
					*current_mode_flow_steps = mode_step_5;
			}
				
			
			break;
		}
		
		//循迹, 只是所定的点不一样
		case mode_step_5:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == circle_connect_line)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);		
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == straight_line || road_state_machine == line_connect_circle_connect_line )
				{
				
					locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y );
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == one_circle)
				{
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);				
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == line_connect_circle)
				{

					
				 {	
						//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
//						if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//						{
//					//进入下一步 
								*current_mode_flow_steps = mode_step_6;
								locate_single_loop_pid_x.increment = 1000;
//								locate_single_loop_pid_y.increment = 0;
//						}
					}

				}
	
				
			}
			else if(fly_direction == backward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
						pitch_offset=1.5;
					if(roll_offset>1)
						roll_offset=1;
					if(roll_offset<-1)
						roll_offset=-1;
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{				
					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
						pitch_offset=1.5;
					if(roll_offset>1)
						roll_offset=1;
					if(roll_offset<-1)
						roll_offset=-1;
				}
				else if(road_state_machine == circle_connect_line)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					
					if(g_position_circle_adjust.x <150 && g_position_circle_adjust.x > -150 
						&& g_position_circle_adjust.y <150 && g_position_circle_adjust.y > -150)
					{
					//进入下一步 
						*current_mode_flow_steps = mode_step_6;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}

				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		

			//当遇到一道杠的时候, 减速
			if(road_state_machine == one_stripes)
			{
				track_pitch_offset_max = 1.8f;
			}
			
/**********************************************************************************************************************/
			//
			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
			//
			
			//对输出进行限幅, 防止速度过快
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}		
		
		case mode_step_6:
		{
			if((road_state_machine == line_connect_circle_connect_line))
			{
						
				*current_mode_flow_steps = mode_step_5;
				
			}
			MODE=6;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//在终点圆上达到稳态, 点亮LED3, 进入等待步
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
				*current_mode_flow_steps = mode_step_7;
				
//				magnet_OFF;
//				set_height_throttle_grab.current_throttle -= 20;
			}
			
				
			
			

			break;
			
		}		
		
		case mode_step_7:
		{
			MODE=7;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//LED3亮3s, 进入返回步
//			if(wait_for_n_ms(&system_time_tick, 300, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_8;
				
				LED3(OFF);
				
				fly_direction = (enum _direction)(1 - fly_direction);
				
				track_pitch_offset_max = 2.0f;
			}

			break;
			
		}		
		
		//返回循迹
		case mode_step_8:
		{
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == circle_connect_line)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
						pitch_offset=1;
					if(roll_offset>1)
						roll_offset=1;
					if(roll_offset<-1)
						roll_offset=-1;
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{						
					locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
						pitch_offset=1;
					if(roll_offset>1)
						roll_offset=1;
					if(roll_offset<-1)
						roll_offset=-1;
				}
				else if(road_state_machine == line_connect_circle)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
					{
					//进入下一步 
					*current_mode_flow_steps = mode_step_9;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}
				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
				
			}
			else if(fly_direction == backward)
			{
							
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					if(g_position_circle_adjust.x <150 && g_position_circle_adjust.x > -150) 
					{
						locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
							pitch_offset=1.8;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
					}
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
						pitch_offset=1.8;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == line_connect_circle_connect_line || road_state_machine == straight_line )
				{					
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_1_adjust.y );
						pitch_offset=1.8;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == one_circle)
				{
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);				
					pitch_offset=1.8;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == circle_connect_line)
				{
				
					{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					//进入下一步 
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100
//					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//					{
					//进入下一步 
					*current_mode_flow_steps = mode_step_9;
						locate_single_loop_pid_x.increment = -1000;
//						locate_single_loop_pid_y.increment = 0;
//					}
					}
				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		

			//当遇到一道杠的时候, 减速
			if(road_state_machine == two_stripes)
			{
				track_pitch_offset_max = 1.8f;
			}
			
/**********************************************************************************************************************/
			//
			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
			//
			
			//对输出进行限幅, 防止速度过快
			
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}		
		
		//在终点圆上方, 进入稳态后可以降落
		case mode_step_9:
		{
			if(road_state_machine == line_connect_circle_connect_line)
			{	
					*current_mode_flow_steps = mode_step_8;
			}
			
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//在终点圆上达到稳态
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
				*current_mode_flow_steps = mode_step_10;
			}

			break;
		}
		
		//降落
		case mode_step_10:
		{
			MODE=10;
			*current_mode_flow_steps = mode_step_11;
			
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;
			
			Suddenly_flag = 1;
			
			break;
		}		
		
		case mode_step_11:
		{
			MODE=11;
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.1, 
										 (g_height-set_height_control_expect_height) * 0.1);


			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 10;
				
				if(g_height < 20 && Suddenly_flag == 1)
				{
					set_height_throttle_grab.current_throttle -= 50;
					Suddenly_flag = 0;
				}
				
				if(set_height_throttle_grab.current_throttle <= 100)
				{
					set_height_throttle_grab.current_throttle = 100;
				}

			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_12;		
				
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_12:
		{
			MODE=12;
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
			
			break;
		}
		
		default:
		{
			
			break;
		}
	}
}

void fly_on_rectangle_control(mode_flow_step_type *current_mode_flow_steps)
{
	float track_pitch_offset_max = 2.2f;
	
	u8 Suddenly_flag = 1;
	

		
	static road_state_machine_type last_road_state_machine = one_stripes;	//此循迹里不会出现的状态

	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			//让偏航角的期望值等于起飞瞬间的yaw值, 这样就不会导致上电后四轴被特意旋转后起飞导致失控
			yaw_offset = angle.yaw;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
			set_height_throttle_grab.current_throttle = 0 ;
			fly_direction = forward;
			magnet_ON;
		
			break;
		}
		
		case mode_step_1:
		{
			MODE=1;
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:
		{
			MODE=2;
//			set_height_control_expect_height = delivery_param_to_control.expect_height_start_circle;
			
			set_height_control_expect_height =90;//高度改低一点图像问题迎刃而解
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
	
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
				if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 3, 100))
				{
					*current_mode_flow_steps = mode_step_3;
					
					//取yaw期望值
//					yaw_offset_tmp = get_yaw_expect(g_position_circle_adjust.x, g_position_circle_adjust.y, 
//													g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				}
				
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_3:
		{
			MODE=3;
//			if(yaw_expect_wave(yaw_offset_tmp, &yaw_offset))
			{
				*current_mode_flow_steps = mode_step_4;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_4:
		{
			MODE=4;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			//进入循迹步
//			if(g_height > set_height_control_expect_height - 5 && g_height < set_height_control_expect_height + 5)
//			{
//				*current_mode_flow_steps = mode_step_5;
//			}
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 8, 80))//定稳后发车
			{
					*current_mode_flow_steps = mode_step_5;
			}
				
			
			break;
		}
		
		//循迹, 只是所定的点不一样
		case mode_step_5:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
					
			
			
			if( road_state_machine == straight_line)		
			{				
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y );
				pitch_offset=-2.0;
//				if(roll_offset>1.2)
//					roll_offset=1.2;
//				if(roll_offset<-1.2)
//					roll_offset=-1.2;
			}
			else if(road_state_machine == line_connect_circle_connect_line ||road_state_machine == one_circle||road_state_machine == Three_line_cricle)
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
				pitch_offset=-2.0;
//				if(roll_offset>1.2)
//					roll_offset=1.2;
//				if(roll_offset<-1.0)
//					roll_offset=-1.0;
			}
			else if(road_state_machine == juxingjiao)		//检测到转角进入下一步 第一个转角
			{
				*current_mode_flow_steps = mode_step_6;
			}
		

			
			break; 
		}

		case mode_step_6://////////////////////////转第一个矩形角
		{
			MODE=6;
			pitch_offset=0;
			
			if(yaw_expect_wave(90, &yaw_offset))//左转
			{
				*current_mode_flow_steps = mode_step_7;
			}
			
			if(yaw_offset>60&&yaw_offset<90)
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				pitch_offset=-1;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
//			if(g_height > 35)
//			{
//				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//			}
//			else
			{
//				roll_offset = 0;
				pitch_offset = 0;
			}
			
		
	
				break;
		}
		
		case mode_step_7:
		{
			MODE=7;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
					
			pitch_offset=-1.6;
			
			if( road_state_machine == straight_line)		
			{				
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y );
				pitch_offset=-2.0;
//				if(roll_offset>1.2)
////					roll_offset=1.2;
				if(roll_offset<-1.2)
					roll_offset=-1.2;
			}
			
			else if(road_state_machine == line_connect_circle_connect_line ||road_state_machine == one_circle||road_state_machine == Three_line_cricle)
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
				pitch_offset=-2.0;
////				if(roll_offset>1.2)
////					roll_offset=1.2;
				if(roll_offset<-1.2)
					roll_offset=-1.2;
			}
			
			else if(road_state_machine == juxingjiao )
			{
				*current_mode_flow_steps = mode_step_8;
			}
			
			
			
			
			break;
		}
		case mode_step_8:   //往左直接飞飞到圆上面  ROLL往左
		{
			MODE=8;

			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			
			if( road_state_machine == Three_line_cricle )
			{
				*current_mode_flow_steps = mode_step_9;
			}
			else
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
				
				roll_offset=-0.8;
				if(pitch_offset>1.0)
					pitch_offset=1.0;
				if(pitch_offset<-1.0)
					pitch_offset=-1.0;
			}
			
			break;
		}
//		case mode_step_8:
//		{
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);

//			
//			if(yaw_expect_wave(90, &yaw_offset))
//			{
//					*current_mode_flow_steps = mode_step_9;
//			}
//			
//			
//			break;
//		}
	
//		case mode_step_9:
//		{
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);

//			pitch_offset=-2.5;
//			roll_offset=0;
//			if(road_state_machine = straight_line)
//			{
//				pitch_offset=-2.0;
////				locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
////				if(roll_offset>1.2)
////					roll_offset=1.2;
////				if(roll_offset<-0.6)
////					roll_offset=-0.6;
//			}
//			else if(road_state_machine == Three_line_cricle)
//			{
//				*current_mode_flow_steps = mode_step_11;
//			}
//			
//			
//			break;
//		}
		case mode_step_9:   //
		{
			MODE=9;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
//				if(roll_offset>3)
//					roll_offset=3;
				if(roll_offset<-1.8)
					roll_offset=-1.8;
				
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			

			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 20, 100))//定稳后发车
			{
					*current_mode_flow_steps = mode_step_10;
			}
			break;
		}
		
		
		case mode_step_10:   //边定点边转向
		{
			MODE=10;
			roll_offset=0;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			
			
			if(fabs(g_position_circle_adjust.x<100) && fabs(g_position_circle_adjust.y)<100)//左转
			{
				if(yaw_expect_wave(90, &yaw_offset))
				{
					*current_mode_flow_steps = mode_step_11;
				}
			}
			
			
			break;
		}
		
		case mode_step_11:   //
		{
			MODE=11;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			if( road_state_machine == straight_line)		
			{				
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y );
				pitch_offset=-1.8;
				if(roll_offset>1.2)
					roll_offset=1.2;
				if(roll_offset<-1.2)
					roll_offset=-1.2;
			}
			else if(road_state_machine == line_connect_circle_connect_line ||road_state_machine == one_circle||road_state_machine == Three_line_cricle)
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
				pitch_offset=-1.8;
//				if(roll_offset>1.2)
//					roll_offset=1.2;
//				if(roll_offset<-1.0)
//					roll_offset=-1.0;
			}
			else if(road_state_machine == juxingjiao)		//检测到转角进入下一步 
			{
				*current_mode_flow_steps = mode_step_13;
			}
			break;
		}
		
//		case mode_step_12:   //
//		{
//			MODE=12;
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
//					
//			
//			
//			if( road_state_machine == straight_line)		
//			{
//				locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				pitch_offset=-1.7;
////				if(roll_offset>1.2)
////					roll_offset=1.2;
//				if(roll_offset<-1.2)
//					roll_offset=-1.2;
//			}
//			else if(road_state_machine == line_connect_circle_connect_line ||road_state_machine == one_circle||road_state_machine == Three_line_cricle)
//			{
//				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
//				pitch_offset=-1.7;
////				if(roll_offset>1.2)
////					roll_offset=1.2;
//				if(roll_offset<-1.2)
//					roll_offset=-1.2;
//			}
//			else if(road_state_machine == juxingjiao )		
//			{
//				*current_mode_flow_steps = mode_step_13;
//			}
//		
//			break;
//		}
		
		case mode_step_13:    //第3个矩形角
		{
			MODE=13;
			pitch_offset=0;
			if(yaw_expect_wave(90, &yaw_offset))//左转
			{
				*current_mode_flow_steps = mode_step_14;
			}
			if(yaw_offset>-120&&yaw_offset<-90&&road_state_machine==straight_line)
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				pitch_offset=-1;
//				if(roll_offset>1.0)
//					roll_offset=1.0;
//				if(roll_offset<-1.0)
//					roll_offset=-1.0;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
//			if(g_height > 35)
//			{
//				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//			}
//			else
			{
//				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_14:   //
		{
			MODE=14;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			pitch_offset=-1.6;
			
			if( road_state_machine == straight_line)
			{				
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				pitch_offset=-2.0;
//				if(roll_offset>1.2)
//					roll_offset=1.2;
//				if(roll_offset<-1.2)
//					roll_offset=-1.2;
			}
			
			else if(road_state_machine == line_connect_circle_connect_line ||road_state_machine == one_circle||road_state_machine == Three_line_cricle)
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
				pitch_offset=-2.0;
//				if(roll_offset>1.2)
//					roll_offset=1.2;
//				if(roll_offset<-1.2)
//					roll_offset=-1.2;
			}
			
			else if(road_state_machine == juxingjiao )
			{
				*current_mode_flow_steps = mode_step_15;
			}
		
			break;
		}
//		case mode_step_13:    //第4个矩形角 我现在不转了
//		{
//				MODE=13;
////			pitch_offset=0;
////			if(yaw_expect_wave(90, &yaw_offset))//左转
////			{
//				*current_mode_flow_steps = mode_step_14;
////			}
////			if(yaw_offset>-30&&yaw_offset<0)
////			{
////				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y);
////			}
//			
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
//			
////			if(g_height > 35)
////			{
////				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
////			}
////			else
//			{
//				roll_offset = 0;
//				pitch_offset = 0;
//			}
//			
//			break;
//		}
		
		case mode_step_15:   //
		{
			MODE=15;
			pitch_offset=0;
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
					
			
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			
			if( road_state_machine == Three_line_cricle )
			{
				*current_mode_flow_steps = mode_step_16;
			}
			else 
			{
				locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_circle_adjust.y);
				if(pitch_offset>0.8)
					pitch_offset=0.8;
				if(pitch_offset<-0.8)
					pitch_offset=-0.8;
				
				roll_offset=-1.0;
			}
		
		
			break;
		}
		
		case mode_step_16:
		{
			MODE=16;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
				if(roll_offset<-1.7)
					roll_offset=-1.7;		
			//在终点圆上达到稳态
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 15, 100))
			{
				*current_mode_flow_steps = mode_step_17;
			}

			break;
		}
		
		//降落
		case mode_step_17:
		{
			MODE=17;
			*current_mode_flow_steps = mode_step_18;
			
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;
			
			Suddenly_flag = 1;
			
			break;
		}
		
		case mode_step_18:
		{
			MODE=18;
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.1, 
										 (g_height-set_height_control_expect_height) * 0.1);


			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 10;
				
				if(g_height < 20 && Suddenly_flag == 1)
				{
					set_height_throttle_grab.current_throttle -= 80;
					Suddenly_flag = 0;
				}
				
				if(set_height_throttle_grab.current_throttle <= 100)
				{
					set_height_throttle_grab.current_throttle = 100;
				}

			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_19;		
				
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_19:
		{
			MODE=19;

/***********************************************************************************************************************************/
			//恢复到原定点 PID
			// The data of pitch
			
/***********************************************************************************************************************************/	
			
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
			
			break;
		}
		
		default:
		{
			
			break;
		}
	}
}


void A_to_B_control(mode_flow_step_type *current_mode_flow_steps)			//加入了三线圆状态
{
	u8 Suddenly_flag = 1;
  
	float track_pitch_offset_max = 2.5f;
	
	if(g_height == 300) g_height =100;
	
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			//让偏航角的期望值等于起飞瞬间的yaw值, 这样就不会导致上电后四轴被特意旋转后起飞导致失控
			yaw_offset = angle.yaw;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
			set_height_throttle_grab.current_throttle = 0 ;
			fly_direction = forward;
			magnet_ON;
		
			break;
		}
		
		case mode_step_1:
		{
			MODE=1;
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:
		{
			MODE=2;
//			set_height_control_expect_height = delivery_param_to_control.expect_height_start_circle;
			set_height_control_expect_height =90;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
	
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
				if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 3, 80))
				{
					*current_mode_flow_steps = mode_step_3;
					
					//取yaw期望值
//					yaw_offset_tmp = get_yaw_expect(g_position_circle_adjust.x, g_position_circle_adjust.y, 
//													g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				}
				
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_3:
		{
			MODE=3;
//			if(yaw_expect_wave(yaw_offset_tmp, &yaw_offset))
			{
				*current_mode_flow_steps = mode_step_4;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_4:
		{
			MODE=4;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			//进入循迹步
//			if(g_height > set_height_control_expect_height - 5 && g_height < set_height_control_expect_height + 5)
//			{
//				*current_mode_flow_steps = mode_step_5;
//			}
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 7, 80))//定稳后发车
			{
					*current_mode_flow_steps = mode_step_5;
			}
				
			
			break;
		}
		
		//循迹, 只是所定的点不一样
		case mode_step_5:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == Three_line_cricle && wait_for_n_ms(&system_time_tick, 600, &x_times_call_wait_for_n_ms_func))
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
//						if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//						{
//					//进入下一步 
								{
									*current_mode_flow_steps = mode_step_6;
									locate_single_loop_pid_x.increment = 1000;
								}
//								locate_single_loop_pid_y.increment = 0;
//						}
				}
				else if(road_state_machine == straight_line )
				{
				
					locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y);
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == one_circle||road_state_machine == Three_line_cricle)
				{
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);				
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}

	
				
			}
			else if(fly_direction == backward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{				
					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
				}
				else if(road_state_machine == circle_connect_line)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					
					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
					{
					//进入下一步 
						*current_mode_flow_steps = mode_step_6;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}
					

				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		

			//当遇到一道杠的时候, 减速
			if(road_state_machine == one_stripes)
			{
				track_pitch_offset_max = 1.8f;
			}
			
/**********************************************************************************************************************/
			//
			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
			//
			
			//对输出进行限幅, 防止速度过快
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}		
		
		case mode_step_6:
		{
//			if((road_state_machine == line_connect_circle_connect_line))
//			{
//				
//				*current_mode_flow_steps = mode_step_5;
//			}
		
			MODE=6;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			magnet_OFF;//开激光
			//在终点圆上达到稳态, 点亮LED3, 进入等待步
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 20, 100))
			{
				*current_mode_flow_steps = mode_step_7;
				
				magnet_OFF;
//				set_height_throttle_grab.current_throttle -= 20;
			}
			
				
			

			break;
			
		}		
		
		case mode_step_7:
		{
			MODE=7;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//LED3亮3s, 进入返回步
//			if(wait_for_n_ms(&system_time_tick, 300, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_9;
				
				LED3(OFF);
				
				fly_direction = (enum _direction)(1 - fly_direction);
				
				track_pitch_offset_max = 1.7f;
			}

			break;
			
		}		
		
//		//返回循迹
//		case mode_step_8:
//		{
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
//			
//			if(fly_direction == forward)
//			{
//				//必须是以下几种路况状态
//				if(road_state_machine == circle_connect_line)
//				{
//					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);					
//				}
//				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
//				{						
//					locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				}
//				else if(road_state_machine == line_connect_circle)
//				{
//					//已经到了终点
//					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//					
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//					{
//					//进入下一步 
//					*current_mode_flow_steps = mode_step_9;
//						locate_single_loop_pid_x.increment = 0;
////						locate_single_loop_pid_y.increment = 0;
//					}
//				}
//				else
//				{
//					//其他是误判的路况状态, 不做处理
//				}
//				
//			}
//			else if(fly_direction == backward)
//			{
//							
//				//必须是以下几种路况状态
//				if(road_state_machine == line_connect_circle)
//				{
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100) 
//					{
//						locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
//						
//					}
//					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
//				}
//				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
//				{					
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				}
//				else if(road_state_machine == circle_connect_line)
//				{
//					//已经到了终点
//					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//					
//					//进入下一步 
////					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100
////					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
////					{
//					//进入下一步 
//					*current_mode_flow_steps = mode_step_9;
//						locate_single_loop_pid_x.increment = -1000;
////						locate_single_loop_pid_y.increment = 0;
////					}

//				}
//				else
//				{
//					//其他是误判的路况状态, 不做处理
//				}
//	
//			}
//			else
//			{
//				//fuck
//			}		

//			//当遇到一道杠的时候, 减速
//			if(road_state_machine == two_stripes)
//			{
//				track_pitch_offset_max = 1.7f;
//			}
//			
///**********************************************************************************************************************/
//			//
//			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
//			//
//			
//			//对输出进行限幅, 防止速度过快
//			
//			track_output_limit(track_pitch_offset_max, &pitch_offset);
//			
//			break; 
//		}		
//		
		//在终点圆上方, 进入稳态后可以降落
		case mode_step_9:
		{
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//在终点圆上达到稳态
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
				*current_mode_flow_steps = mode_step_10;
			}

			break;
		}		
		
		//降落
		case mode_step_10:
		{
			MODE=10;
			*current_mode_flow_steps = mode_step_11;
			
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;   //
			
			Suddenly_flag = 1;
			
			break;
		}		
		
		case mode_step_11:
		{
			MODE=11;
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.1, 
										 (g_height-set_height_control_expect_height) * 0.1);


			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 10;
				
				if(g_height < 20 && Suddenly_flag == 1)
				{
					set_height_throttle_grab.current_throttle -= 50;
					Suddenly_flag = 0;
				}
				
				if(set_height_throttle_grab.current_throttle <= 100)
				{
					set_height_throttle_grab.current_throttle = 100;
				}

			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_12;		
				magnet_ON;     //关激光
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_12:
		{
			MODE=12;
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
			
			break;
		}
		
		default:
		{
			
			break;
		}
	}
	

}

void A_C_B_control(mode_flow_step_type *current_mode_flow_steps)
{
	u8 Suddenly_flag = 1;
  
	float track_pitch_offset_max = 2.5f;
	
	if(g_height == 300) g_height =110;
	
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			//让偏航角的期望值等于起飞瞬间的yaw值, 这样就不会导致上电后四轴被特意旋转后起飞导致失控
			yaw_offset = angle.yaw;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
			set_height_throttle_grab.current_throttle = 0 ;
			fly_direction = forward;
			magnet_ON;
		
			break;
		}
		
		case mode_step_1:
		{
			MODE=1;
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:
		{
			MODE=2;
//			set_height_control_expect_height = delivery_param_to_control.expect_height_start_circle;
			set_height_control_expect_height =110;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
	
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
				if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 3, 80))
				{
					*current_mode_flow_steps = mode_step_3;
					
					//取yaw期望值
//					yaw_offset_tmp = get_yaw_expect(g_position_circle_adjust.x, g_position_circle_adjust.y, 
//													g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				}
				
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_3:
		{
			MODE=3;
//			if(yaw_expect_wave(yaw_offset_tmp, &yaw_offset))
			{
				*current_mode_flow_steps = mode_step_4;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_4:
		{
			MODE=4;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			//进入循迹步
//			if(g_height > set_height_control_expect_height - 5 && g_height < set_height_control_expect_height + 5)
//			{
//				*current_mode_flow_steps = mode_step_5;
//			}
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 7, 80))//定稳后发车
			{
					*current_mode_flow_steps = mode_step_5;
			}
				
			
			break;
		}
		
		//循迹, 只是所定的点不一样
		case mode_step_5:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == circle_connect_line)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);		
					pitch_offset=-2.2;
//					if(roll_offset>2.2)
//						roll_offset=2.2;
//					if(roll_offset<-2.2)
//						roll_offset=-2.2;
				}
				else if(road_state_machine == straight_line )
				{
				
					locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y );
					pitch_offset=-2.2;
//					if(roll_offset>2.2)
//						roll_offset=2.2;
//					if(roll_offset<-2.2)
//						roll_offset=-2.2;
				}
				else if(road_state_machine == one_circle)
				{
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);				
					pitch_offset=-2.2;
//					if(roll_offset>2.2)
//						roll_offset=2.2;
//					if(roll_offset<-2.2)
//						roll_offset=-2.2;
				}
				else if(road_state_machine == line_connect_circle_connect_line)//到了中间的C点 线圆线
				{

					
					//已经到了中间的C点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
//						if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//						{
//					//进入下一步 
								{
									*current_mode_flow_steps = mode_step_6;
									locate_single_loop_pid_x.increment = 1000;
								}
//								locate_single_loop_pid_y.increment = 0;
//						}
					

				}
	
				
			}
			else if(fly_direction == backward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{				
					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
				}
				else if(road_state_machine == circle_connect_line)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					
					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
					{
					//进入下一步 
						*current_mode_flow_steps = mode_step_6;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}
					

				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		


			

			
			//对输出进行限幅, 防止速度过快
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}		
		
		case mode_step_6:
		{
			MODE=6;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(road_state_machine == straight_line )
			{
				*current_mode_flow_steps = mode_step_5;
					
			}
			//在终点圆上达到稳态, 进入转向步
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 100))
			{
				*current_mode_flow_steps = mode_step_7;
				
				magnet_OFF;
//				set_height_throttle_grab.current_throttle -= 20;
			}
			
				
			

			break;
			
		}
		
		case mode_step_7:
		{
			MODE=7;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//转向 指向B点
			if(fabs(g_position_circle_adjust.x)<100&&fabs(g_position_circle_adjust.y)<100)
			{
				if(yaw_expect_wave(-85, &yaw_offset))
				{
					*current_mode_flow_steps = mode_step_8;//转完方向启动下一步
					fly_direction = forward;
				}
			}


			break;
			
		}		
		
		//循迹, 只是所定的点不一样
		case mode_step_8:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle_connect_line || road_state_machine == one_circle)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);		
					pitch_offset=-2.2;
//					if(roll_offset>2.2)
//						roll_offset=2.2;
//					if(roll_offset<-2.2)
//						roll_offset=-2.2;
				}
				else if(road_state_machine == straight_line )
				{
				
					locate_single_loop_pid_control(g_position_line_1_adjust.x, g_position_line_1_adjust.y);
					pitch_offset=-2.2;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
	
				else if(road_state_machine == line_connect_circle)//到了B点 线连圆
				{

					
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
//						if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//						{
//					//进入下一步 
								{
									*current_mode_flow_steps = mode_step_9;
									locate_single_loop_pid_x.increment = 1000;
								}
//								locate_single_loop_pid_y.increment = 0;
//						}
					

				}
	
				
			}
			else if(fly_direction == backward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{				
					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
				}
				else if(road_state_machine == circle_connect_line)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					
					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
					{
					//进入下一步 
						*current_mode_flow_steps = mode_step_6;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}
					

				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		


			

			
			//对输出进行限幅, 防止速度过快
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}	
		case mode_step_9:
		{
			MODE=7;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//LED3亮3s, 进入返回步
//			if(wait_for_n_ms(&system_time_tick, 300, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_10;
				
//				LED3(OFF);
//				
//				fly_direction = (enum _direction)(1 - fly_direction);
//				
//				track_pitch_offset_max = 1.7f;
			}

			break;
			
		}		
		
//		//返回循迹
//		case mode_step_8:
//		{
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
//			
//			if(fly_direction == forward)
//			{
//				//必须是以下几种路况状态
//				if(road_state_machine == circle_connect_line)
//				{
//					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);					
//				}
//				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
//				{						
//					locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				}
//				else if(road_state_machine == line_connect_circle)
//				{
//					//已经到了终点
//					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//					
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//					{
//					//进入下一步 
//					*current_mode_flow_steps = mode_step_9;
//						locate_single_loop_pid_x.increment = 0;
////						locate_single_loop_pid_y.increment = 0;
//					}
//				}
//				else
//				{
//					//其他是误判的路况状态, 不做处理
//				}
//				
//			}
//			else if(fly_direction == backward)
//			{
//							
//				//必须是以下几种路况状态
//				if(road_state_machine == line_connect_circle)
//				{
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100) 
//					{
//						locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
//						
//					}
//					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
//				}
//				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
//				{					
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				}
//				else if(road_state_machine == circle_connect_line)
//				{
//					//已经到了终点
//					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//					
//					//进入下一步 
////					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100
////					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
////					{
//					//进入下一步 
//					*current_mode_flow_steps = mode_step_9;
//						locate_single_loop_pid_x.increment = -1000;
////						locate_single_loop_pid_y.increment = 0;
////					}

//				}
//				else
//				{
//					//其他是误判的路况状态, 不做处理
//				}
//	
//			}
//			else
//			{
//				//fuck
//			}		

//			//当遇到一道杠的时候, 减速
//			if(road_state_machine == two_stripes)
//			{
//				track_pitch_offset_max = 1.7f;
//			}
//			
///**********************************************************************************************************************/
//			//
//			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
//			//
//			
//			//对输出进行限幅, 防止速度过快
//			
//			track_output_limit(track_pitch_offset_max, &pitch_offset);
//			
//			break; 
//		}		
//		
		//在终点圆上方, 进入稳态后可以降落
		case mode_step_10:
		{
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//在终点圆上达到稳态
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
				*current_mode_flow_steps = mode_step_11;
			}

			break;
		}		
		
		//降落
		case mode_step_11:
		{
			MODE=10;
			*current_mode_flow_steps = mode_step_12;
			
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;   //
			
			Suddenly_flag = 1;
			
			break;
		}		
		
		case mode_step_12:
		{
			MODE=11;
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.1, 
										 (g_height-set_height_control_expect_height) * 0.1);


			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 10;
				
				if(g_height < 20 && Suddenly_flag == 1)
				{
					set_height_throttle_grab.current_throttle -= 50;
					Suddenly_flag = 0;
				}
				
				if(set_height_throttle_grab.current_throttle <= 100)
				{
					set_height_throttle_grab.current_throttle = 100;
				}

			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_13;		
				
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_13:
		{
			MODE=12;
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
			
			break;
		}
		
		default:
		{
			
			break;
		}
	}
	

}


void FIND_LCUK_control(mode_flow_step_type *current_mode_flow_steps)
{	
	u8 Suddenly_flag = 1;
  
	float track_pitch_offset_max = 2.5f;
	
	if(g_height == 300) g_height =100;
	
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			//让偏航角的期望值等于起飞瞬间的yaw值, 这样就不会导致上电后四轴被特意旋转后起飞导致失控
			yaw_offset = angle.yaw;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
			set_height_throttle_grab.current_throttle = 0 ;
			fly_direction = forward;
			magnet_ON;
		
			break;
		}
		
		case mode_step_1:
		{
			MODE=1;
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:
		{
			MODE=2;
//			set_height_control_expect_height = delivery_param_to_control.expect_height_start_circle;
			set_height_control_expect_height =90;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
	
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
				if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 3, 80))
				{
					*current_mode_flow_steps = mode_step_3;
					
					//取yaw期望值
//					yaw_offset_tmp = get_yaw_expect(g_position_circle_adjust.x, g_position_circle_adjust.y, 
//													g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				}
				
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_3:
		{
			MODE=3;
//			if(yaw_expect_wave(yaw_offset_tmp, &yaw_offset))
			{
				*current_mode_flow_steps = mode_step_4;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_4:
		{
			MODE=4;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			//进入循迹步
//			if(g_height > set_height_control_expect_height - 5 && g_height < set_height_control_expect_height + 5)
//			{
//				*current_mode_flow_steps = mode_step_5;
//			}
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 7, 80))//定稳后发车
			{
					*current_mode_flow_steps = mode_step_5;
			}
				
			
			break;
		}
		
		//循迹, 只是所定的点不一样
		case mode_step_5:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == circle_connect_line)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);		
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == straight_line )
				{
				
					locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == one_circle)
				{
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);				
					pitch_offset=-2.7;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == line_connect_circle)
				{

					
						//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
//						if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//						{
//					//进入下一步 
								{
									*current_mode_flow_steps = mode_step_6;
									locate_single_loop_pid_x.increment = 1000;
								}
//								locate_single_loop_pid_y.increment = 0;
//						}
					

				}
	
				
			}
			else if(fly_direction == backward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{				
					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
				}
				else if(road_state_machine == circle_connect_line)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					
					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
						&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
					{
					//进入下一步 
						*current_mode_flow_steps = mode_step_6;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}
					

				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		

			//当遇到一道杠的时候, 减速
			if(road_state_machine == one_stripes)
			{
				track_pitch_offset_max = 1.8f;
			}
			
/**********************************************************************************************************************/
			//
			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
			//
			
			//对输出进行限幅, 防止速度过快
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}		
		
		case mode_step_6:
		{
//			if((road_state_machine == line_connect_circle_connect_line))
//			{
//				
//				*current_mode_flow_steps = mode_step_5;
//			}
		
			MODE=6;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			magnet_OFF;//开激光
			//在终点圆上达到稳态, 点亮LED3, 进入等待步
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 20, 100))
			{
				*current_mode_flow_steps = mode_step_7;
				
				magnet_OFF;
//				set_height_throttle_grab.current_throttle -= 20;
			}
			
				
			

			break;
			
		}		
		
		case mode_step_7:
		{
			MODE=7;
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//LED3亮3s, 进入返回步
//			if(wait_for_n_ms(&system_time_tick, 300, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_9;
				
				LED3(OFF);
				
				fly_direction = (enum _direction)(1 - fly_direction);
				
				track_pitch_offset_max = 2.3f;
			}

			break;
			
		}		
		
//		//返回循迹
//		case mode_step_8:
//		{
//			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
//			
//			if(fly_direction == forward)
//			{
//				//必须是以下几种路况状态
//				if(road_state_machine == circle_connect_line)
//				{
//					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);					
//				}
//				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
//				{						
//					locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				}
//				else if(road_state_machine == line_connect_circle)
//				{
//					//已经到了终点
//					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//					
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
//					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//					{
//					//进入下一步 
//					*current_mode_flow_steps = mode_step_9;
//						locate_single_loop_pid_x.increment = 0;
////						locate_single_loop_pid_y.increment = 0;
//					}
//				}
//				else
//				{
//					//其他是误判的路况状态, 不做处理
//				}
//				
//			}
//			else if(fly_direction == backward)
//			{
//							
//				//必须是以下几种路况状态
//				if(road_state_machine == line_connect_circle)
//				{
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100) 
//					{
//						locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
//						
//					}
//					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
//				}
//				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
//				{					
//					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
//				}
//				else if(road_state_machine == circle_connect_line)
//				{
//					//已经到了终点
//					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
//					
//					//进入下一步 
////					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100
////					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
////					{
//					//进入下一步 
//					*current_mode_flow_steps = mode_step_9;
//						locate_single_loop_pid_x.increment = -1000;
////						locate_single_loop_pid_y.increment = 0;
////					}

//				}
//				else
//				{
//					//其他是误判的路况状态, 不做处理
//				}
//	
//			}
//			else
//			{
//				//fuck
//			}		

//			//当遇到一道杠的时候, 减速
//			if(road_state_machine == two_stripes)
//			{
//				track_pitch_offset_max = 1.7f;
//			}
//			
///**********************************************************************************************************************/
//			//
//			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
//			//
//			
//			//对输出进行限幅, 防止速度过快
//			
//			track_output_limit(track_pitch_offset_max, &pitch_offset);
//			
//			break; 
//		}		
//		
		//在终点圆上方, 进入稳态后可以降落
		case mode_step_9:
		{
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//在终点圆上达到稳态
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
				*current_mode_flow_steps = mode_step_10;
			}

			break;
		}		
		
		//降落
		case mode_step_10:
		{
			MODE=10;
			*current_mode_flow_steps = mode_step_11;
			
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;   //
			
			Suddenly_flag = 1;
			
			break;
		}		
		
		case mode_step_11:
		{
			MODE=11;
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.1, 
										 (g_height-set_height_control_expect_height) * 0.1);


			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 10;
				
				if(g_height < 20 && Suddenly_flag == 1)
				{
					set_height_throttle_grab.current_throttle -= 50;
					Suddenly_flag = 0;
				}
				
				if(set_height_throttle_grab.current_throttle <= 100)
				{
					set_height_throttle_grab.current_throttle = 100;
				}

			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_12;		
				magnet_ON;     //关激光
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
//				ARMED = 0;
			}
			break;
		}
		
		case mode_step_12:
		{
			MODE=12;
			*current_mode_flow_steps = mode_step_none;
			
//			ARMED=1;
			mode_machine = B_to_A;			
			
			break;
		}
		
		default:
		{
			
			break;
		}
	}
}


//

void B_to_A_control(mode_flow_step_type *current_mode_flow_steps)
{	
	u8 Suddenly_flag = 1;
  
	float track_pitch_offset_max = 2.5f;
	
	if(g_height == 300) g_height =110;
	
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			//让偏航角的期望值等于起飞瞬间的yaw值, 这样就不会导致上电后四轴被特意旋转后起飞导致失控
			yaw_offset = angle.yaw;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
			set_height_throttle_grab.current_throttle = 0 ;
			fly_direction = backward;
			magnet_ON;
		
			break;
		}
		
		case mode_step_1:
		{
			MODE=1;
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 300, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:
		{
			MODE=2;
//			set_height_control_expect_height = delivery_param_to_control.expect_height_start_circle;
			set_height_control_expect_height =90;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
	
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				
				if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 3, 80))
				{
					*current_mode_flow_steps = mode_step_3;
					
					//取yaw期望值
//					yaw_offset_tmp = get_yaw_expect(g_position_circle_adjust.x, g_position_circle_adjust.y, 
//													g_position_line_1_adjust.x, g_position_line_1_adjust.y);
				}
				
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_3:
		{
			MODE=3;
//			if(yaw_expect_wave(yaw_offset_tmp, &yaw_offset))
			{
				*current_mode_flow_steps = mode_step_4;
			}
			
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			break;
		}
		
		case mode_step_4:
		{
			MODE=4;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			//进入循迹步
//			if(g_height > set_height_control_expect_height - 5 && g_height < set_height_control_expect_height + 5)
//			{
//				*current_mode_flow_steps = mode_step_5;
//			}
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 7, 80))//定稳后发车
			{
					*current_mode_flow_steps = mode_step_5;
			}
				
			
			break;
		}
		
		//返回循迹
		case mode_step_5:
		{
			MODE=5;
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			if(fly_direction == forward)
			{
				//必须是以下几种路况状态
				if(road_state_machine == circle_connect_line)
				{
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_line_2_adjust.y);
						pitch_offset=1;
					if(roll_offset>1)
						roll_offset=1;
					if(roll_offset<-1)
						roll_offset=-1;
				}
				else if(road_state_machine == straight_line || road_state_machine == two_stripes || road_state_machine == one_stripes)
				{						
					locate_single_loop_pid_control(g_position_line_1_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
						pitch_offset=1;
					if(roll_offset>1)
						roll_offset=1;
					if(roll_offset<-1)
						roll_offset=-1;
				}
				else if(road_state_machine == line_connect_circle)
				{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100 
					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
					{
					//进入下一步 
					*current_mode_flow_steps = mode_step_9;
						locate_single_loop_pid_x.increment = 0;
//						locate_single_loop_pid_y.increment = 0;
					}
				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
				
			}
			else if(fly_direction == backward)
			{
							
				//必须是以下几种路况状态
				if(road_state_machine == line_connect_circle)
				{
					if(g_position_circle_adjust.x <150 && g_position_circle_adjust.x > -150) 
					{
						locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
							pitch_offset=2.0;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
					}
					//机头不论是朝向何方, 都定位 line_2.end, 数据在F405中已经处理完毕
					locate_single_loop_pid_control(g_position_line_2_adjust.x, g_position_circle_adjust.y);
						pitch_offset=2.0;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == line_connect_circle_connect_line || road_state_machine == straight_line )
				{					
					locate_single_loop_pid_control(g_position_line_2_adjust.x, (g_position_line_1_adjust.y + g_position_line_2_adjust.y) / 2.0f);
						pitch_offset=2.0;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == one_circle)
				{
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);				
					pitch_offset=2.0;
//					if(roll_offset>1.2)
//						roll_offset=1.2;
//					if(roll_offset<-1.2)
//						roll_offset=-1.2;
				}
				else if(road_state_machine == circle_connect_line)
				{
				
					{
					//已经到了终点
					locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
					
					//进入下一步 
//					if(g_position_circle_adjust.x <100 && g_position_circle_adjust.x > -100
//					&& g_position_circle_adjust.y <100 && g_position_circle_adjust.y > -100)
//					{
					//进入下一步 
					*current_mode_flow_steps = mode_step_6;
						locate_single_loop_pid_x.increment = -1000;
//						locate_single_loop_pid_y.increment = 0;
//					}
					}
				}
				else
				{
					//其他是误判的路况状态, 不做处理
				}
	
			}
			else
			{
				//fuck
			}		

			//当遇到一道杠的时候, 减速
			if(road_state_machine == two_stripes)
			{
				track_pitch_offset_max = 1.8f;
			}
			
/**********************************************************************************************************************/
			//
			//实测从两道杠到邻近圆的时间非常短, 减速效果不明显
			//
			
			//对输出进行限幅, 防止速度过快
			
			track_output_limit(track_pitch_offset_max, &pitch_offset);
			
			break; 
		}		
		
		//在终点圆上方, 进入稳态后可以降落
		case mode_step_6:
		{
			if(road_state_machine == line_connect_circle_connect_line)
			{	
					*current_mode_flow_steps = mode_step_8;
			}
			
			locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
			set_height_cascade_control_2(set_height_control_expect_height, set_height_control_expect_height/5, set_height_control_expect_height/6);
			
			//在终点圆上达到稳态
			if(stady_state_judge(g_position_circle_adjust, set_point_expect_position, 10, 80))
			{
				*current_mode_flow_steps = mode_step_7;
			}

			break;
		}
		
		//降落
		case mode_step_7:
		{
			MODE=10;
			*current_mode_flow_steps = mode_step_8;
			
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;
			
			Suddenly_flag = 1;
			
			break;
		}		
		
		case mode_step_8:
		{
			MODE=11;
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.1, 
										 (g_height-set_height_control_expect_height) * 0.1);


			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 10;
				
				if(g_height < 20 && Suddenly_flag == 1)
				{
					set_height_throttle_grab.current_throttle -= 50;
					Suddenly_flag = 0;
				}
				
				if(set_height_throttle_grab.current_throttle <= 100)
				{
					set_height_throttle_grab.current_throttle = 100;
				}

			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_9;		
				
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_9:
		{
			MODE=12;
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
			
			break;
		}
		
		default:
		{
			
			break;
		}
	}
}






void set_height_turning_control(mode_flow_step_type *current_mode_flow_steps)//定高控制函数
{
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			MODE=0;
			*current_mode_flow_steps = mode_step_1;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
		
			break;
		}
		case mode_step_1:
		{
			
			
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		case mode_step_2:    								//停在这一步，下一步等上位机指令（降落）
		{
			MODE=2;
			set_height_control_expect_height = 90;
			set_height_cascade_control_2(set_height_control_expect_height, 
																		set_height_control_expect_height/5, 
																		set_height_control_expect_height/6);
			
			
			if(g_height >(set_height_control_expect_height-10)&&wait_for_n_ms(&system_time_tick, 800, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_3;
			}
			
			break;
		}
		
		case mode_step_3:      							//降落
		{
			set_height_cascade_control_2(set_height_control_expect_height, 70, 50);

			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;
			
			*current_mode_flow_steps = mode_step_4;
		}
		
		case mode_step_4:
		{
			set_height_cascade_control_2(set_height_control_expect_height, (g_height-set_height_control_expect_height)*0.3, (g_height-set_height_control_expect_height) * 0.15);
		
			if(g_height < 30)
			{
					set_height_throttle_grab.current_throttle -= 4;
			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_5;		
				
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
				
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_5:
		{
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
		}
		
		default:
		{
			
			break;
		}
	}
}


void set_point_turning_control(mode_flow_step_type *current_mode_flow_steps)
{
	switch(*current_mode_flow_steps)
	{
		case mode_step_none:
		{
			*current_mode_flow_steps = mode_step_1;
			
			set_height_height_grab.flag = 0;	
			set_height_throttle_grab.flag = 0;
		
			break;
		}
		case mode_step_1:
		{
		  
			//等待8s
			if(wait_for_n_ms(&system_time_tick, 1000, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_2;
				increment_set_to_zero();
			}
			
			break;
		}
		
		case mode_step_2:    //定高
		{
//			set_height_control_expect_height = 110;
//			set_height_cascade_control_2(set_height_control_expect_height, 25, 15);

			set_height_cascade_control_2(	set_height_control_expect_height, 
																		set_height_control_expect_height/5, 
																		set_height_control_expect_height/6);
			
			if(g_height > 35)      //高度大于25开始定点
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);//定点PID
			}
			else
			{
				roll_offset = 0;
				pitch_offset = 0;
			}
			
			if(g_height >(set_height_control_expect_height-10)&&wait_for_n_ms(&system_time_tick, 1900, &x_times_call_wait_for_n_ms_func))
			{
				*current_mode_flow_steps = mode_step_3;
			}
//			if(g_height >(set_height_control_expect_height-10)&&fabs(g_position_circle_adjust.x)<60&&fabs(g_position_circle_adjust.y)<60)
//			{
//				
//				if(yaw_expect_wave(90, &yaw_offset))
//				{
//					*current_mode_flow_steps = mode_step_3;
//				}
//			}
			break;
		}
		
		

		
		
		
		
		case mode_step_3:
		{
			*current_mode_flow_steps = mode_step_4;
		
			set_height_height_grab.flag = 0;	
			
			set_height_control_expect_height = -60;
		
			//降落PID
			
		}
		
		case mode_step_4:
		{
			set_height_cascade_control_2(set_height_control_expect_height, 
										 (g_height-set_height_control_expect_height) * 0.2, 
										 (g_height-set_height_control_expect_height) * 0.1);
		
			
			if(g_height > 30)
			{
				locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
				//高度补偿, 做进函数
			}
			else
			{
				//降落、消除定点作用
				pitch_offset = 0;
				roll_offset = 0;
				locate_single_loop_pid_x.increment = 0;
				locate_single_loop_pid_y.increment = 0;	
				
				//直接减油门, 快速降落
				set_height_throttle_grab.current_throttle -= 4;
				
				if(set_height_throttle_grab.current_throttle  <= 10)
				{
					set_height_throttle_grab.current_throttle = 0;
				}
			}

			if(g_height < 10)
			{
				*current_mode_flow_steps = mode_step_5;		
				
				ctrl.height.core.pid_out = 0;
				ctrl.height.shell.pid_out = 0;
				ctrl.height.shell.increment = 0;
				ctrl.height.core.increment = 0;
			
				ARMED = 0;
			}
			break;
		}
		
		case mode_step_5:
		{
			*current_mode_flow_steps = mode_step_none;
		
			mode_machine = mode_none;			
		}
		
		default:
		{
			
			break;
		}
	}
}
//typedef enum _mode_flow_step
//{
//	mode_step_none,
//	mode_step_1,
//	mode_step_2,
//	mode_step_3,
//	mode_step_4,
//	mode_step_5,
//	mode_step_6,
//	mode_step_7,
//	mode_step_8,
//	mode_step_9,
//	mode_step_10,
//	mode_step_11,
//	mode_step_12,
//	mode_step_13,
//	mode_step_14,
//	mode_step_15,
//	mode_step_16,
//}mode_flow_step_type;

//等待+起飞+旋转+降落...
mode_flow_step_type mode_1_flow_steps;

//强制自动降落
mode_flow_step_type mode_2_flow_steps;

//定高测试
mode_flow_step_type mode_3_flow_steps;

//定点测试...
mode_flow_step_type mode_4_flow_steps;

//循迹测试...
mode_flow_step_type mode_5_flow_steps;

//A_to_B_fire_B_to_A
mode_flow_step_type mode_A_to_B_fire_B_to_A_flow_steps;

//飞矩形
mode_flow_step_type mode_fly_on_rectangle_flow_steps;

//A_to_B
mode_flow_step_type mode_A_to_B_steps;

//LUCK
mode_flow_step_type mode_FIND_LCUK_steps;

//B_to_A
mode_flow_step_type mode_B_to_A_steps;

//A_C_B
mode_flow_step_type mode_A_C_B_steps;

//其他模式...
mode_flow_step_type mode_6_flow_steps;

//typedef enum _mode_machine
//{
//	mode_none,	
//	gesture_turning,
//	force_to_auto_land,
//	wait_setheight_spin_land,
//	wait_setheight_setpoint_spin_track_setpoint_land,
//	set_height_turning,
//	set_point_turning,
//	
//	A_to_B_fire_B_to_A,		//7
//	
//	fly_on_rectangle,
//	
//	A_to_B,
//	
//	FIND_LCUK,
//	
//	B_to_A,
//	
//	set_height_,
//	
//}mode_machine_type;

void business_mode_handle(mode_machine_type *mode_machine)//题目要求模式状态机
{
	if(!ARMED)
	{
		*mode_machine = mode_none;
//		locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);
	}
	
	switch(*mode_machine)
	{
		case mode_none:
		{

			break;
		}
		

		case set_height_:
		{
			set_height_turning_control(&mode_3_flow_steps);  //进入定高控制
		
			break;
		}			
		
		case set_point_turning:      //进入定点控制
		{
			set_point_turning_control(&mode_4_flow_steps);
		
			break;
		}

		case A_to_B:     							//AB国赛
		{
			A_to_B_control(&mode_A_to_B_steps);
			
			break;
		}

		
		case A_to_B_fire_B_to_A:  		//ABA圆-线-圆
		{
			A_to_B_fire_B_to_A_control(&mode_A_to_B_fire_B_to_A_flow_steps);
			
			break;
		}

		case fly_on_rectangle:   		//矩形-国赛
		{
			fly_on_rectangle_control(&mode_fly_on_rectangle_flow_steps);
			
			break;
		}
		
		case FIND_LCUK:  //A-B(降落起飞)-B
		{
			FIND_LCUK_control(&mode_FIND_LCUK_steps);
			
			break;
		}
		
		//紧接 FIND_LCUK
		case B_to_A:
		{
			B_to_A_control(&mode_B_to_A_steps);
			
			break;
		}
		
		case A_C_B:     //15年校赛ACB
		{
			A_C_B_control(&mode_A_C_B_steps);
			
			break;
		}

		default: 
		{
			break;
		}
	}
	
	//将其他模式下的流程步骤归零
	mode_flow_step_set_to_zero(mode_machine);
	
	//将其他模式下的控制积分、输出清零
	integral_output_set_to_zero(mode_machine);
	
	//testing...
	test_report_data_s16_1 = mode_FIND_LCUK_steps;
	
}

//
//@brief :	更新上次飞行状态, 道路状态 和  变量x_times_call_wait_for_n_ms_func
//
//
//
void update_last_mode_state(const enum _mode_machine current_mode, enum _mode_machine *last_mode,
							const enum _road_state_machine current_road_state, enum _road_state_machine *last_road_state,
							u16 *x_times_call_wait_for_n_ms_func)
{
	//当当前飞行模式和上次飞行模式不同时，更新x_times_call_wait_for_n_ms_func
	if(current_mode != *last_mode)
	{
		(*x_times_call_wait_for_n_ms_func)++;
		
	}
	
	
	//以下的状态更新在有wait延时函数时就他妈的是个炸弹
	
	//更新上次飞行状态
	*last_mode = current_mode;
	
	//更新上次道路状态
	*last_road_state = current_road_state;
}



void mode_flow_step_set_to_zero(const mode_machine_type *mode_machine)
{
	switch(*mode_machine)
	{
		case mode_none:
		{
			mode_1_flow_steps = mode_step_none;
			mode_2_flow_steps = mode_step_none;
			mode_3_flow_steps = mode_step_none;
			mode_4_flow_steps = mode_step_none;
			mode_5_flow_steps = mode_step_none;
			
			mode_A_to_B_fire_B_to_A_flow_steps = mode_step_none;
			mode_fly_on_rectangle_flow_steps = mode_step_none;
			mode_A_to_B_steps = mode_step_none;
			
			mode_FIND_LCUK_steps = mode_step_none;
			mode_B_to_A_steps = mode_step_none;
			
			mode_A_C_B_steps = mode_step_none;
			break;
		}
		
		default: 
		{
			break;
		}
	}
}


void increment_set_to_zero(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment = 0;
	ctrl.yaw.shell.increment = 0;

	ctrl.pitch.core.increment = 0;
	ctrl.roll.core.increment = 0;
	ctrl.yaw.core.increment = 0;

	
	ctrl.height.shell.increment = 0;
	ctrl.height.core.increment = 0;
	
	pitch_offset = 0;
	roll_offset = 0;
	
	ctrl.locate_x.shell.increment = 0;
	ctrl.locate_y.shell.increment = 0;

	ctrl.locate_x.core.increment = 0;
	ctrl.locate_y.core.increment = 0;
	
	locate_single_loop_pid_x.increment = 0;
	locate_single_loop_pid_y.increment = 0;

}


void integral_output_set_to_zero(const mode_machine_type *mode_machine)
{
	switch(*mode_machine)
	{
		case mode_none:
		{
			ctrl.pitch.shell.increment = 0;
			ctrl.roll.shell.increment = 0;
			ctrl.yaw.shell.increment = 0;

			ctrl.pitch.core.increment = 0;
			ctrl.roll.core.increment = 0;
			ctrl.yaw.core.increment = 0;

			
			ctrl.height.shell.increment = 0;
			ctrl.height.core.increment = 0;
			
			pitch_offset = 0;
			roll_offset = 0;
			
			ctrl.locate_x.shell.increment = 0;
			ctrl.locate_y.shell.increment = 0;

			ctrl.locate_x.core.increment = 0;
			ctrl.locate_y.core.increment = 0;
			
			break;
		}
		
		
		default: 
		{
			break;
		}

	}
		
}



