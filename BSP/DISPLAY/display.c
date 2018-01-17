#include "display.h"
#include "include.h"

float BATT[20];


enum _dispaly_data display_what;



void State_Display(void)
{
	OLED_Fill(0x00); //清屏
	

  OLED_P6x8Str(0,0,"pit:");
	OLED_P6x8Str(0,1,"pit:");
	OLED_P6x8Str(0,2,"pit:");

	OLED_P6x8Str(0,4,"roll-p:");
	OLED_P6x8Str(0,5,"roll-i:");
	OLED_P6x8Str(0,6,"roll-d:");
	
	OLED_P6x8Str(70,0,"yaw:");
	OLED_P6x8Str(70,1,"yaw:");
	OLED_P6x8Str(70,2,"yaw:");
	
  
}


void  parameterDispaly(void)   //pid调整参数显示
{
	
 
	
//	OLED_Num5(4, 5, system_time_tick / 10);
	
OLED_P6x8Str(0,0,"pit:");
	OLED_P6x8Str(0,1,"pit:");
	OLED_P6x8Str(0,2,"pit:");

	OLED_P6x8Str(0,4,"roll-p:");
	OLED_P6x8Str(0,5,"roll-i:");
	OLED_P6x8Str(0,6,"roll-d:");
	
	OLED_P6x8Str(70,0,"yaw:");
	OLED_P6x8Str(70,1,"yaw:");
	OLED_P6x8Str(70,2,"yaw:");
	
  Dis_Float(0, 20, ctrl.pitch.shell.kp, 4);			
  Dis_Float(1, 20, ctrl.pitch.shell.ki, 4);			
  Dis_Float(2, 20, ctrl.pitch.shell.kd,4);	
	
	Dis_Float(4, 40, ctrl.roll.shell.kp, 4);			
  Dis_Float(5, 40, ctrl.roll.shell.ki, 4);			
  Dis_Float(6, 40, ctrl.roll.shell.kd,4);	
	
	Dis_Float(0, 90, ctrl.yaw.shell.kp, 4);			
  Dis_Float(1, 90, ctrl.yaw.shell.ki, 4);			
  Dis_Float(2, 90, ctrl.yaw.shell.kd, 4);	
	
	OLED_P6x8Str(78,5,"shell");

}

void display_imgae(u8 *camera_image_buf)
{

	if(new_image_flag == came)
	{
		Draw_BMP(0, 0, 80, 6, camera_image_buf); 
		
		new_image_flag = coming;	
		
		get_image_or_position_data(get_image);
		
	}
}



