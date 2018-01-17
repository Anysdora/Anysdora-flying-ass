#include "include.h"
#include "hci.h"


struct _hci quad_hci = {level_zero};
enum _key_function KeyVal;
/**enum _key_function
{
	KEY_NONE,
	KEY_ENTER,
	KEY_EXIT,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_START,
};*/

struct _delivery_param delivery_param_to_control = {80};

extern int k;
u8 zhuangtai_num=0;

u8 qiankui_num=0;

void hci(void)
{
	
	KeyVal = Key_Scan();          //KEY处理
	
	switch(KeyVal)
	{
		
		case KEY_START:            //KEY1
		{
			k=0;
	    OLED_Fill(0x00); //清屏	
			set_height_control_expect_height+=10;
			if(set_height_control_expect_height==210)
			{
				set_height_control_expect_height=50;
			}
			Dis_Float(0, 10, set_height_control_expect_height,2);
			
			break;
		}
		
		case KEY_EXIT:            //KEY2
		{
			k=0;
			zhuangtai_num++;
			if(zhuangtai_num>7)
			{
				zhuangtai_num=0;
				
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"mode_none:");
				
			}
			
			if(zhuangtai_num==1)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"Dingdian:");
				
			}
			else if(zhuangtai_num==2)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"A_to_B:");
				
			}
			else if(zhuangtai_num==3)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"A_to_B_fire_B_to_A:");	
				
			}
			else if(zhuangtai_num==4)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"A_C_B:");	

			}
			else if(zhuangtai_num==5)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"fly_on_rectangle:");	

			}
			else if(zhuangtai_num==6)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"FIND_LCUK:");	
			
			}
			else if(zhuangtai_num==7)
			{
				OLED_P6x8Str(0,7, "                    ");
				OLED_P6x8Str(0,7,"set_height_turning:");	
			
			}
			
			ARMED=0;//上锁
			

			
			break;
		}
		
		case KEY_ENTER:            //KEY3
		{
			k=0;
									
			if(zhuangtai_num==1)
			{
				
				OLED_P6x8Str(70,7,"Fly.....:");	
				ARMED=1;
				mode_machine = set_point_turning;
			}
			else if(zhuangtai_num==2)
			{
				
				OLED_P6x8Str(70,7,"Fly.....:");
				ARMED=1;
				mode_machine = A_to_B;
			}
			else if(zhuangtai_num==3)
			{
				
				OLED_P6x8Str(70,7,"Fly.....:");
				ARMED=1;
				mode_machine = A_to_B_fire_B_to_A;
			}
			else if(zhuangtai_num==4)
			{
				
				OLED_P6x8Str(70,7,"Fly.....:");
				ARMED=1;
				mode_machine = A_C_B;
			}
			else if(zhuangtai_num==5)
			{
					
				OLED_P6x8Str(70,7,"Fly.....:");	
				ARMED=1;
				mode_machine = fly_on_rectangle;
			}
			else if(zhuangtai_num==6)
			{
				
				OLED_P6x8Str(70,7,"Fly.....:");	
				ARMED=1;
				mode_machine = FIND_LCUK;
			}
			else if(zhuangtai_num==7)
			{
					
				OLED_P6x8Str(70,7,"Fly.....:");	
				ARMED=1;
				mode_machine = set_height_;
			}
			
			break;
		}
		case KEY_RIGHT:            //KEY4
		{
			k=0;
			
      qiankui_num++;
			
		
				OLED_Fill(0x00); //清屏	
				OLED_P6x8Str(0,0,"moto 12:");	
				Dis_Float(0, 70, gesture_feedforward_moto_1, 1);
				Dis_Float(1, 70, gesture_feedforward_moto_2, 1);
			
			
				OLED_P6x8Str(0,2,"moto 01:");	
				Dis_Float(2, 70, gesture_feedforward_moto_0, 1);
				Dis_Float(3, 70, gesture_feedforward_moto_1, 1);
			 
			
				OLED_P6x8Str(0,5,"pitch:");	
				Dis_Float(5, 70, steady_state_pitch_expect, 1);
			
		
				OLED_P6x8Str(0,6,"roll:");	
				Dis_Float(6, 70, steady_state_roll_expect, 1);
			
				if(qiankui_num>4)
					qiankui_num=1;
			
			
			break;
		}
		case KEY_DOWN:            //KEY5 ++
		{
			k=0;
			if(qiankui_num==1)
			{				
				gesture_feedforward_moto_1++;
				gesture_feedforward_moto_2++;
				Dis_Float(0, 70, gesture_feedforward_moto_1, 1);
				Dis_Float(1, 70, gesture_feedforward_moto_2, 1);
			}
			else if(qiankui_num==2)
			{				
				gesture_feedforward_moto_0++;
				gesture_feedforward_moto_1++;
				Dis_Float(2, 70, gesture_feedforward_moto_0, 1);
				Dis_Float(3, 70, gesture_feedforward_moto_1, 1);
			}
			else if(qiankui_num==3)
			{
				steady_state_pitch_expect+=0.1;
				Dis_Float(5, 70, steady_state_pitch_expect, 1);
			}
			else if(qiankui_num==4)
			{
				steady_state_roll_expect+=0.1;
				Dis_Float(6, 70, steady_state_roll_expect, 1);
			}
			
				break;
		}
		
		case KEY_LEFT:            //KEY6 --
		{
			k=0;
			
			if(qiankui_num==1)
			{				
				gesture_feedforward_moto_1--;
				gesture_feedforward_moto_2--;
				Dis_Float(0, 70, gesture_feedforward_moto_1, 1);
				Dis_Float(1, 70, gesture_feedforward_moto_2, 1);
			}
			else if(qiankui_num==2)
			{				
				gesture_feedforward_moto_0--;
				gesture_feedforward_moto_1--;
				Dis_Float(2, 70, gesture_feedforward_moto_0, 1);
				Dis_Float(3, 70, gesture_feedforward_moto_1, 1);
			}
			else if(qiankui_num==3)
			{
				steady_state_pitch_expect-=0.1;
				Dis_Float(5, 70, steady_state_pitch_expect, 1);
			}
			else if(qiankui_num==4)
			{
				steady_state_roll_expect-=0.1;
				Dis_Float(6, 70, steady_state_roll_expect, 1);
			}
			
			
				break;
		}
		default:
		{
			
			break;
		}
		
		
	}
}




