#include "TIM_PWM_IN.h"

T_RC_DATA Rc_Data;//1000~2000

void TIM4_Cap_Init(void)
{	 
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM4_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	 //使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO功能的时钟
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);  //进行重映射

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;             //PB6 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;            
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);		

	//初始化定时器4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =71; 	                 //预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM4输入捕获参数
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM_Cmd(TIM4,ENABLE ); 

	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);	
	
/////////////////////////////////////////////////////////////////////////////////初始化定时器8 TIM8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	 //使能TIM8时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIOC时钟
	
	
	/*  TIM8 channel 1 pin (PC.06) configuration */ 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
 	GPIO_Init(GPIOC, &GPIO_InitStructure); 
		
	 TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                      //设定计数器自动重装值 
	 TIM_TimeBaseStructure.TIM_Prescaler =71; 	                 //预分频器   
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	 TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
   
	  //初始化TIM8输入捕获参数

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM8, &TIM4_ICInitStructure);
//	TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);        //????????

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM8, &TIM4_ICInitStructure);
//	TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);        //??????????
		
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM8, &TIM4_ICInitStructure);
//	TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);        //?????????
		
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM8, &TIM4_ICInitStructure);
//	TIM_SelectMasterSlaveMode(TIM8,TIM_MasterSlaveMode_Enable);        //??????????
		
    TIM_ITConfig(TIM8,TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM8,TIM_IT_CC2, ENABLE);   
	TIM_ITConfig(TIM8,TIM_IT_CC3, ENABLE); 
	TIM_ITConfig(TIM8,TIM_IT_CC4, ENABLE); 
	TIM_Cmd(TIM8,ENABLE ); 	
}


u8  TIM4CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM4CH1_CAPTURE_VAL;	  //输入捕获值

u8  TIM4CH2_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM4CH2_CAPTURE_VAL;	  //输入捕获值

u8  TIM4CH3_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM4CH3_CAPTURE_VAL;	  //输入捕获值

u8  TIM4CH4_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM4CH4_CAPTURE_VAL;	  //输入捕获值

//定时器4中断服务程序	 
void TIM4_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); //清除中断标志位
			if(TIM4CH1_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
			{	  			
				TIM4CH1_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
				TIM4CH1_CAPTURE_VAL=TIM_GetCapture1(TIM4);
		   	TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								                                 //还未开始,第一次捕获上升沿
			{
				TIM4CH1_CAPTURE_STA=0;			                         //清空
				TIM4CH1_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM4,0);
				TIM4CH1_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}	
	 
	  if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2); //清除中断标志位
			if(TIM4CH2_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
			{	  			
				TIM4CH2_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
				TIM4CH2_CAPTURE_VAL=TIM_GetCapture2(TIM4);
		   		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								                                 //还未开始,第一次捕获上升沿
			{
				TIM4CH2_CAPTURE_STA=0;			                         //清空
				TIM4CH2_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM4,0);
				TIM4CH2_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}	
  	
    if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); //清除中断标志位
			if(TIM4CH3_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
			{	  			
				TIM4CH3_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
				TIM4CH3_CAPTURE_VAL=TIM_GetCapture3(TIM4);
		   	TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								                                 //还未开始,第一次捕获上升沿
			{
				TIM4CH3_CAPTURE_STA=0;			                         //清空
				TIM4CH3_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM4,0);
				TIM4CH3_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}	

    if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); //清除中断标志位
			if(TIM4CH4_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
			{	  			
				TIM4CH4_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
				TIM4CH4_CAPTURE_VAL=TIM_GetCapture4(TIM4);
		   		TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								                                 //还未开始,第一次捕获上升沿
			{
				TIM4CH4_CAPTURE_STA=0;			                         //清空
				TIM4CH4_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM4,0);
				TIM4CH4_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		    TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}		
}

//获取4路PWM值
void GET_FOUR_PWM(void)
{
   if(TIM4CH1_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
		Rc_Data.YAW = TIM4CH1_CAPTURE_VAL;        //得到总的高电平时
		TIM4CH1_CAPTURE_STA=0;                //开启下一次捕获
	 }
			
	 if(TIM4CH2_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
		Rc_Data.THROTTLE = TIM4CH2_CAPTURE_VAL;  //得到总的高电平时
		TIM4CH2_CAPTURE_STA=0;                //开启下一次捕获
	 }
			
	 if(TIM4CH3_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
		Rc_Data.PITCH = TIM4CH3_CAPTURE_VAL;        //得到总的高电平时
		TIM4CH3_CAPTURE_STA=0;                //开启下一次捕获
	 }
			
	 if(TIM4CH4_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
		Rc_Data.ROLL = TIM4CH4_CAPTURE_VAL;        //得到总的高电平时
		TIM4CH4_CAPTURE_STA=0;                //开启下一次捕获
	 }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
u8  TIM8CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM8CH1_CAPTURE_VAL;	  //输入捕获值
u8  TIM8CH2_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM8CH2_CAPTURE_VAL;	  //输入捕获值
u8  TIM8CH3_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM8CH3_CAPTURE_VAL;	  //输入捕获值
u8  TIM8CH4_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM8CH4_CAPTURE_VAL;	  //输入捕获值

u16 Rc_Data_T8_C1;	
u16 Rc_Data_T8_C2;
u16 Rc_Data_T8_C3;  //0--1087, 1--1487, 2--1887 上下浮动在1、2之间
u16 Rc_Data_T8_C4;

//定时器8cc中断服务程序
void TIM8_CC_IRQHandler(void)
{		
	if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); //清除中断标志位
		if(TIM8CH1_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
		{
			TIM8CH1_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
			TIM8CH1_CAPTURE_VAL=TIM_GetCapture1(TIM8);
		TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
		}
		else  								                                 //还未开始,第一次捕获上升沿
		{
			TIM8CH1_CAPTURE_STA=0;			                         //清空
			TIM8CH1_CAPTURE_VAL=0;
			TIM_SetCounter(TIM8,0);
			TIM8CH1_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		TIM_OC1PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
		}
	}

	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)            //捕获2发生捕获事件
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);               //清除中断标志位
		if(TIM8CH2_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
		{
			TIM8CH2_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
			TIM8CH2_CAPTURE_VAL=TIM_GetCapture2(TIM8);
		TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
		}
		else  								                                 //还未开始,第一次捕获上升沿
		{
			TIM8CH2_CAPTURE_STA=0;			                         //清空
			TIM8CH2_CAPTURE_VAL=0;
			TIM_SetCounter(TIM8,0);
			TIM8CH2_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		TIM_OC2PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC2P=1 设置为下降沿捕获
		}
	}
	
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)            //捕获3发生捕获事件
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);               //清除中断标志位
		if(TIM8CH3_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
		{
			TIM8CH3_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
			TIM8CH3_CAPTURE_VAL=TIM_GetCapture3(TIM8); 
		TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
		}
		else  								                                 //还未开始,第一次捕获上升沿
		{
			TIM8CH3_CAPTURE_STA=0;			                         //清空
			TIM8CH3_CAPTURE_VAL=0;
			TIM_SetCounter(TIM8,0);
			TIM8CH3_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		TIM_OC3PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC2P=1 设置为下降沿捕获
		}
	}
	
	
	if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4); //清除中断标志位
		if(TIM8CH4_CAPTURE_STA&0X40)		                       //捕获到一个下降沿 		
		{
			TIM8CH4_CAPTURE_STA|=0X80;		                       //标记成功捕获到一次上升沿
			TIM8CH4_CAPTURE_VAL=TIM_GetCapture4(TIM8);
		TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
		}
		else  								                                 //还未开始,第一次捕获上升沿
		{
			TIM8CH4_CAPTURE_STA=0;			                         //清空
			TIM8CH4_CAPTURE_VAL=0;
			TIM_SetCounter(TIM8,0);
			TIM8CH4_CAPTURE_STA|=0X40;		                        //标记捕获到了上升沿
		TIM_OC4PolarityConfig(TIM8,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
		}
	}		
}

 void GET_TIM8_PWM(void)
{
  	  if(TIM8CH1_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
			Rc_Data_T8_C1 = TIM8CH1_CAPTURE_VAL;        //得到总的高电平时
			TIM8CH1_CAPTURE_STA=0;  
			            //开启下一次捕获
	 }  
	 
	 if(TIM8CH2_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
			Rc_Data_T8_C2 = TIM8CH2_CAPTURE_VAL;        //得到总的高电平时
			TIM8CH2_CAPTURE_STA=0;                //开启下一次捕获
	 }
	  if(TIM8CH3_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
			Rc_Data_T8_C3 = TIM8CH3_CAPTURE_VAL;        //得到总的高电平时
			TIM8CH3_CAPTURE_STA=0;  
			            //开启下一次捕获
	 }  
	  if(TIM8CH4_CAPTURE_STA&0X80)          //成功捕获到了一次上升沿
	 {
			Rc_Data_T8_C4 = TIM8CH4_CAPTURE_VAL;        //得到总的高电平时
			TIM8CH4_CAPTURE_STA=0;                //开启下一次捕获
	 }
}




