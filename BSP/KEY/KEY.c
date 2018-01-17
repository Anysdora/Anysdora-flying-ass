#include "key.h"
#include "include.h"

void KEY_Init(void)      //KEY1-6~PE8-13  
{
	GPIO_InitTypeDef  KEY_GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(KEY_RCCEN, ENABLE); 	 

	KEY_GPIO_InitStructure.GPIO_Pin = KEY1|KEY2|KEY3|KEY4|KEY5|KEY6;
	KEY_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	KEY_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(KEY_GPIO, &KEY_GPIO_InitStructure); 		      
}

 /*
 * 函数名：Key_Scan
 * 描述  ：检测是否有按键按下
 * 输入  ：GPIOx：x 可以是 A，B，C，D或者 E
 *		     GPIO_Pin：待读取的端口位 	
 * 输出  ：KEY_OFF(没按下按键)、KEY_ON（按下按键）
 */
enum _key_function Key_Scan(void)
{			
	/*检测是否有按键按下 */
	if(GPIO_ReadInputDataBit(KEY_GPIO,KEY1) == KEY_ON ) 
	{	   
		delay_ms(5);		
		if(GPIO_ReadInputDataBit(KEY_GPIO,KEY1) == KEY_ON )  
		{	 
			while(GPIO_ReadInputDataBit(KEY_GPIO,KEY1) == KEY_ON);   
			return 	KEY_START;			
		}
	}
	else if(GPIO_ReadInputDataBit(KEY_GPIO,KEY2) == KEY_ON ) 
	{	   
		delay_ms(5);		
		if(GPIO_ReadInputDataBit(KEY_GPIO,KEY2) == KEY_ON )  
		{	 
			while(GPIO_ReadInputDataBit(KEY_GPIO,KEY2) == KEY_ON);   
			return 	KEY_EXIT;	 
		}
	}
	else if(GPIO_ReadInputDataBit(KEY_GPIO,KEY3) == KEY_ON ) 
	{	   
		delay_ms(5);		
		if(GPIO_ReadInputDataBit(KEY_GPIO,KEY3) == KEY_ON )  
		{	 
			while(GPIO_ReadInputDataBit(KEY_GPIO,KEY3) == KEY_ON);   
			return 	KEY_ENTER;	 
		}
	}
	else if(GPIO_ReadInputDataBit(KEY_GPIO,KEY4) == KEY_ON ) 
	{	   
		delay_ms(5);		
		if(GPIO_ReadInputDataBit(KEY_GPIO,KEY4) == KEY_ON )  
		{	 
			//连续按键
			while(GPIO_ReadInputDataBit(KEY_GPIO,KEY4) == KEY_ON)
			{
				delay_ms(140);
				
				return 	KEY_RIGHT;	 	
			}				
			return 	KEY_RIGHT;	 
		}
	}
	else if(GPIO_ReadInputDataBit(KEY_GPIO,KEY5) == KEY_ON ) 
	{	   
		delay_ms(5);		
		if(GPIO_ReadInputDataBit(KEY_GPIO,KEY5) == KEY_ON )  
		{	 
			while(GPIO_ReadInputDataBit(KEY_GPIO,KEY5) == KEY_ON);   
			return 	KEY_DOWN;
		}
	}
	else if(GPIO_ReadInputDataBit(KEY_GPIO,KEY6) == KEY_ON ) 
	{	   
		delay_ms(5);		
		if(GPIO_ReadInputDataBit(KEY_GPIO,KEY6) == KEY_ON )  
		{	 
			//连续按键
			while(GPIO_ReadInputDataBit(KEY_GPIO,KEY6) == KEY_ON)
			{
				delay_ms(140);
				
				return 	KEY_LEFT;	 	
			}
			return 	KEY_LEFT;
		}
	}
	else 
	{
		return KEY_NONE;
	}
	
	extern enum _key_function KeyVal;
	
	if(KeyVal!=KEY_START&&KeyVal!=KEY_EXIT&&KeyVal!=KEY_ENTER)
	return KEY_NONE;
}


