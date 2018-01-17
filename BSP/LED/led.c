#include "led.h"
#include "include.h"

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_Init(void)   //LED1-PB10    LED2-PB11    LED3-PD10    LED4-PD11
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);     //PB

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_10 | GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);     //PD

	GPIO_SetBits(GPIOB,GPIO_Pin_10 | GPIO_Pin_11);	 // turn off all led
	GPIO_SetBits(GPIOD,GPIO_Pin_10 | GPIO_Pin_11);	 // turn off all led	
}

void LED_Sailing(void)
{
	static u8 r;
	
	r++;
	if(r >> 2)        // r/4 == r>>2
	{
		LED1(ON);
		r = 0;
	}
	else
	{
		LED1(OFF);
	}
}

//led显示处理
void led_handle(void)
{
	//解锁灯处理
	if(ARMED)	 
	{
		LED1(ON);
	}
	else
	{
		LED1(OFF);
	}			

	
}
