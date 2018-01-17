#ifndef __ULTRASONIC_H_
#define __ULTRASONIC_H_

#include "include.h"

//四轴高度
extern float g_height;


extern u8 g_height_H;
extern u8 g_height_L;

//探测指令重发标志
extern u8 g_KS103_retrans_flag;

enum _flag
{
	high,
	low 
};

extern enum _flag g_HL_flag;

void send_sonar_cmd(void);

//高度数据滤波处理  滑动滤波
float sonar_slide_filter(float height);

//发送超声波降噪指令
//延时1s
//该指令将在下一次上电时起作用
void ultrasonic_init(void);


//在main文件中合成高度数据
void height_composite(float *height);


#endif
