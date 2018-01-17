#include "include.h"
#include "init.h"


void IAC_Init(void)
{
	delay_init(72);		//延时函数初始化 ok
	Nvic_Init();		//中断初始化 ok
	LED_Init();			//led初始化  ok
	OLED_Init();		//oled初始化 ok
	//Draw_LibLogo();   //画LOGO ok
	ADC1_Init();		//AD初始化   ok
	I2C_INIT();			//IIC初始化  ok   PB6 PB7
	KEY_Init();             //   ok


	
	Moto_Init();		  //电机初始化     ok
	usart1_config(); 	//串口1初始化 --- 摄像头 115200 ok    PA9 PA10
	usart2_config();	//串口2初始化 --- 上位机 115200 ok    PD5 PD6                               
	usart3_config(); 	//串口3初始化 --- 超声波 9600   ok    PD8 PD9
	ultrasonic_init();	//该超声波降噪处理须在定时器初始化之前，降噪未处理完成不能发送探测指令
	TIM3_Init(10000); 	//定时器初始化 10MS
	NRF24L01_Init();
	NRF24L01_Mode(1);	//设置2401为接收模式	
	
	delay_ms(500);
	
	mpu_dmp_init();		//dmp初始化 100hz

}


