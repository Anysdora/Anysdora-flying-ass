#ifndef	_HEIGHT_H_
#define _HEIGHT_H_

#include"include.h"

#define height_base 4.0f

enum _height_mode
{
	manually,						
	auto_takeoff,
	set_height,
	auto_land,
};

extern enum _height_mode height_mode;

extern struct _pid auto_land_pid;


//定高期望值
extern float set_height_control_expect_height;



//微分先行中防止第一次运算出现bug
extern float auto_land_last_height;

//自动降落
void auto_land_control(void);

//遥控定高通道判断 + 定高模式判断
void height_mode_judge(void);


struct _throttle_grab
{
	u8 flag;
	int current_throttle;
};
extern struct _throttle_grab auto_land_throttle_grab;
extern struct _throttle_grab set_height_throttle_grab;


//_height_grab.flag == 0说明尚未抓取高度
struct _height_grab
{
	u8 flag;
	float current_height;
};
extern struct _height_grab auto_land_height_grab;
extern struct _height_grab set_height_height_grab;



//高度模式事务处理
void height_mode_handle(void);

//自动降落PID给定
void auto_land_pid_set(void);


//定高函数, 模块化设计
void set_height_cascade_control_2(float expect_height, u16 high_speed_time, u16 low_speed_time);

#endif


