#include "ultrasonic.h"

//四轴高度
float g_height = 0;
float g_height_original;

//串口2接收来自超声波探测到的高度的高低字节
u8 g_height_H = 0;
u8 g_height_L = 0;

//当前高低字节标志
enum _flag g_HL_flag;

//重发指令标志
u8 g_KS103_retrans_flag = 1;			//1:表示可重发探测指令

//通过串口3向超声波发送探测指令,同时清零高低字节,清零高低字节标志
//0xe8+0x02+0xb4
//0XB4 : 0-5m范围，带温度补偿，返回mm,探测耗时约87ms.
void send_sonar_cmd(void)
{
	while((USART3->SR&0X40)==0);
	USART3->DR = 0XE8;
	delay_us(80);
	while((USART3->SR&0X40)==0);
	
	USART3->DR = 0X02;
	delay_us(80);
	while((USART3->SR&0X40)==0);
	
	USART3->DR = 0XB4;
	delay_us(80);
	while((USART3->SR&0X40)==0);
		
	g_HL_flag = high;
}

//高度数据滤波处理 
//限幅 ，突变， 滑动滤波
#define data_N 5
float sonar_slide_filter(float height)
{
	static float yuesefu[10] = {0};
	static char cursor = 0;
	float ans = 0;
	static float ans_old = 0;
	
	//数据限幅
	if((height > 350) || (height < 0))
	{
		return ans_old;
	}
	
	yuesefu[cursor] = height;
	
	cursor++;
	if(data_N == cursor)
	{
		cursor = 0;
	}
	
	for(int i = 0; i < data_N; i++)
	{
		ans += yuesefu[i];
	}
	
	ans /= data_N;
	
	//解决数据突变问题
	if(ans > 350 || ans < 0)
	{
		ans = ans_old;
	}
	
	ans_old = ans;
	
	return ans;
}

//发送超声波降噪指令
//延时1s
//该指令将在下一次上电时起作用
void ultrasonic_init(void)
{
	while((USART3->SR&0X40)==0);
	USART3->DR = 0XE8;
	delay_us(80);
	while((USART3->SR&0X40)==0);
	
	USART3->DR = 0X02;
	delay_us(80);
	while((USART3->SR&0X40)==0);
	
	USART3->DR = 0X75;						//最高级别降噪
	delay_us(80);
	while((USART3->SR&0X40)==0);
	
	delay_ms(1500);
}


#define data_N_2 5
float sonar_accel_slide_limit_filter(float height)
{
	static float loop[data_N_2] = {0};
	static u8 cursor = 0;
	
	float ans = 0;
	static float last_ans = 0;
	
	static float last_height = 0;
	static float last_single_height;
	
	//数据限幅
	if((height > 250) || (height < 0))
	{
		return last_ans;
	}
	
	//下降最大差值不超过40cm(自由落体运动)
	if(last_height - height > 40)
	{
		return last_ans;
	}
	//上升最大差值不超过40cm(加速度计的量程为+-2g)
	//存在飞机突然抽疯，加速上升，此时最大差值可能超过40cm
	if(height - last_height > 40)
	{
		//依然发疯
		if(height - last_height > 60)
		{
			return 300;
		}
		return last_ans;
	}
	
	if( height > 60 && height - last_single_height > 4) 
	{
		height = height * 0.05 + 0.95 * last_single_height; 
	}
	last_single_height = height; 
	
	loop[cursor] = height;
	
	cursor = (cursor + 1) % data_N_2;
	
	for(int i = 0; i < data_N_2; i++)
	{
		ans += loop[i];
	}
	ans /= data_N_2;
	
	last_ans = ans;
	last_height = ans;
	
	return ans;
}


void height_composite(float *height)//height=&g_height
{
	//合成高度数据	单位:cm		
	 g_height_original = *height = ((g_height_H << 8) | g_height_L) / 10.0 ;
	*height = sonar_accel_slide_limit_filter(g_height);
}

