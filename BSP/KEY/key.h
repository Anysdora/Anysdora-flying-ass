#ifndef _KEY_H
#define _KEY_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"

//µË
//#define KEY_GPIO    GPIOE
//#define KEY_RCCEN   RCC_APB2Periph_GPIOE
//#define KEY1        GPIO_Pin_8      
//#define KEY2        GPIO_Pin_9     
//#define KEY3        GPIO_Pin_10      
//#define KEY4        GPIO_Pin_11       
//#define KEY5        GPIO_Pin_12       
//#define KEY6        GPIO_Pin_13

//#define KEY_ON	  	0	
//#define KEY_OFF		1

//±ó
#define KEY_GPIO    GPIOE
#define KEY_RCCEN   RCC_APB2Periph_GPIOE
#define KEY1        GPIO_Pin_15      
#define KEY2        GPIO_Pin_12     
#define KEY3        GPIO_Pin_10      
#define KEY4        GPIO_Pin_9       
#define KEY5        GPIO_Pin_8       
#define KEY6        GPIO_Pin_7

#define KEY_ON	  	0	
#define KEY_OFF		1
//#define KEY_ENTER	2
//#define KEY_EXIT	3
//#define KEY_UP  	4
//#define KEY_DOWN	5
//#define KEY_LEFT	6
//#define KEY_RIGHT	7
//#define KEY_START	8


enum _key_function
{
	KEY_NONE,
	KEY_ENTER,
	KEY_EXIT,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_START,
};



void KEY_Init(void);

enum _key_function Key_Scan(void);

void key_handle(void);



#endif

