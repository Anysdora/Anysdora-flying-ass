#include "nvic.h"
#include "include.h"
//优先级顺序从先——后为TIM4（遥控）   TIM8     USART_1（摄像头）   USART_3（超声波）  TIM3（中断） 24L01
//                                   USART_2（上位机） 

void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; //定义一个结构体变量 NVIC_InitStructure
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;             //TIM3
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;            //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);                            //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;         //TIM8  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
	NVIC_Init(&NVIC_InitStructure);                            


	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	         //USART1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	         //USART2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	         //USART3
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the NRF24L01 PC3 EXTI */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;					 //PC3 通道3端口
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	 //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		 		 //亚优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					   //使能中断
	NVIC_Init(&NVIC_InitStructure);
}

