#include "include.h" 
int k=1;

extern int16_t MODE;

int main(void)
{
	IAC_Init();     	//外设及驱动配置
	paramLoad();		//参数加载  前馈参数和各种PID参数
	//State_Display();	//OLED数据显示
	EnTIM3();       	//开定时器 飞行模式的识别 姿态控制 上锁解锁处理
	
	magnet_ON;
	
//	magnet_OFF;//开激光
	while(1)
	{    
		//DMP-->标准100HZ, 几乎不受定时器或其他代码执行的影响8
		//新移植代码, 未测试
		inv_mpu_read(&angle, &sensor);
					
		//每20MS向上位机发送一次数据
		//每20MS合成一次数据(高度)
		if(sentDateFlag)  		//向上位机发送数据标志
		{
/**********************************************************************************************************/	
			sentDateFlag = 0;			
		
			
			//串口发送姿态
			UART2_ReportIMU();


			//高度数据合成，滤波
			height_composite(&g_height);

			//位置数据合成，校正，滤波
			camera_data_image_decode(USART_ov7620_rxbuf, &camera_RX_BUF_point);
/*
      P/1000   I/10000   D/100 传姿态pid数据
      Data_Receive_Pid1(&locate_single_loop_pid_x, &locate_single_loop_pid_y, &scapegoat_1);		//kp/1000
  		Data_Receive_Pid1(&ctrl.roll.core, &ctrl.pitch.core, &ctrl.yaw.core);		//kp/1000  内环
		 	Data_Receive_Pid2(&ctrl.roll.shell, &ctrl.pitch.shell, &ctrl.yaw.shell);		//kp/1000  外环
			Data_Receive_Pid2(&ctrl.height.shell, &ctrl.height.core, &ctrl.yaw.shell);		//kp/1000  外环
	
			//此函数异常重要，脱控后上锁解锁就通过此函数解码上位机发送过来的指令
			Data_Receive_lock(&ARMED);
		
*/	
								
				//向摄像头模块发送高度信息(即获取位置、状态等数据的指令)				
				if(display_what == param)
				{
					get_image_or_position_data(get_position_data);
				}
/**********************************************************************************************************/					
				//向超声波模块发送获取高度指令
				if(1 == g_KS103_retrans_flag)
				{
					send_sonar_cmd();

					g_KS103_retrans_flag = 0;
				}	

				led_handle();	
				
				if(!ARMED || g_height < 10 )
				{
					if(k)
					{
//						OLED_P6x8Str(10,0,"pitch:");
//						OLED_P6x8Str(10,1,"roll:");
//						OLED_P6x8Str(10,2,"yaw:");
//						OLED_P6x8Str(10,5,"Heigh:");
				
						Dis_Float(0, 10, angle.pitch,2);			
						Dis_Float(1, 10, angle.roll, 2);			
						Dis_Float(2, 10, angle.yaw,  2);						
						Dis_Float(4, 10, g_height,   2);
//						Dis_Float(6, 10, MODE,   2);
						Dis_Float(6, 10, road_state_machine,   2);
						Dis_Float(2, 65, g_position_circle_receive.y, 1);
						Dis_Float(3, 65, g_position_circle_receive.x, 1);
//						
//						Dis_Float(4, 65, (g_position_line_1_adjust.y+g_position_line_2_adjust.y)/2.0f, 1);
//						Dis_Float(5, 65, g_position_line_1_adjust.y, 1);
//						Dis_Float(6, 65, g_position_line_2_adjust.y, 1);
//						
						Dis_Float(5, 65, g_position_line_1_adjust.y, 1);
						Dis_Float(6, 65, g_position_line_2_adjust.y, 1);
						
						Dis_Float(7, 65, g_position_line_1_receive.x, 1);

//						Dis_Float(2, 65, g_position_circle_adjust.x, 1);
//						Dis_Float(3, 65, g_position_circle_adjust.y, 1);
						
						
//						locate_single_loop_pid_control(g_position_circle_adjust.x, g_position_circle_adjust.y);//定点PID
//						Dis_Float(5, 65, pitch_offset, 1);
//						Dis_Float(6, 65, roll_offset, 1);
						
						
					}
					
					
					hci();                          //重点看，hci（）为人机交互代码
				}
				
			}
	}
}

