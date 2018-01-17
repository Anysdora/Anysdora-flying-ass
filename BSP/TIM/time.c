#include "time.h"
#include "stm32f10x.h"


/**************************实现函数********************************************
*函数原型:		
*功　　能:1ms中断一次,计数器为1000		
*******************************************************************************/
void TIM3_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructure.TIM_Period=period_num;//装载值
	
	//prescaler is 72,that is 72000000/72 = 1000000Hz 即 1us;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//分频系数
	
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	//clear the TIM3 overflow interrupt flag
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	//TIM3 overflow interrupt enable
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	//enable TIM3
	TIM_Cmd(TIM3,DISABLE);
}


/**************************实现函数******************************************************			
*功　　能:1ms计数一次，通过get_ms_for_6050dmp(unsigned long *count)传递计数值给MPU6050DMP
*说    明:没有中断
*****************************************************************************************/
void TIM1_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_DeInit(TIM1);

	TIM_TimeBaseStructure.TIM_Period = 0XFFFF;//装载值
	
	//prescaler is 35999,that is 72000000/(35999 + 1)/2=1000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=35999;//分频系数
	
	//set clock division 
	//此处设置为 2时钟分频， 应为prescaler设置为0xffff依然无法达到1ms计数一次
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV2; //or TIM_CKD_DIV1 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	//clear the TIM3 overflow interrupt flag
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	
	//TIM3 overflow interrupt disable
	TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);
	
	//enable TIM1
	TIM_Cmd(TIM1,ENABLE);
}

//此函数用于DMP6050的DMP的时间戳，然而并没有什么卵用
void  get_ms_gansha(unsigned long *count)
{
	*count = TIM1->CNT;
	
	TIM1->CNT = 0;
}
