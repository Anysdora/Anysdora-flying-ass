#include "include.h"
#include "stm32f10x_it.h"


//系统时间滴, 频率100HZ
u32 system_time_tick = 0;


_distance distance = {0, 0, 0};//陀螺仪的 distance
_speed speed = {0, 0, 0};      //陀螺仪XYZ方向的speed

u8 sentDateFlag = 0;
 
void TIM3_IRQHandler(void)		    //10ms中断一次
{
	static u16 TIM3_expand_1=0;	//中断次数计数器1
	static u16 TIM3_expand_2=0;	//中断次数计数器2

	if(TIM3->SR & TIM_IT_Update)
	{
		TIM3->SR = ~TIM_FLAG_Update;//清除中断标志		
		TIM3_expand_1++;
		TIM3_expand_2++;
		
		//系统时钟滴, 每10ms增加一次
		system_time_tick++;
		
		
		X_CONTROL(angle.roll, angle.pitch, angle.yaw);// 10ms更新一次姿态

	
//		注意以下函数先后顺序
				
		if(TIM3_expand_2 == 4)				//10 * 4 = 40ms更新一次飞行模式
		{
			TIM3_expand_2 = 0;

			//飞行模式判断
			business_mode_handle(&mode_machine);

			//更新上次飞行状态, 道路状态 和 变量x_times_call_wait_for_n_ms_func, system_time_tick
			update_last_mode_state(mode_machine, &mode_machine_last,
							   road_state_machine, &road_state_machine_last, 
							   &x_times_call_wait_for_n_ms_func); 
		}

		//上锁解锁处理
		if(TIM3_expand_1 == 2)				//10 * 2 = 20ms中断一次
		{
			TIM3_expand_1 = 0;
			
			out_of_control_lock(&angle);//倾角过大上锁
			
			sentDateFlag = 1;
		}

		//电机输出处理
		MOTO_output_handle();
	}
}

