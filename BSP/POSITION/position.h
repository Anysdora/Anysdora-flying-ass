#ifndef	_POSITION_H_
#define _POSITION_H_

#include "include.h"


//因为摄像头并不在四轴的正中心, 而且图像窗口也不是正中心, 所以为了使四轴的中心对准圆, 改变期望值
extern struct _position_float set_point_expect_position;



extern float pitch_offset;
extern float roll_offset;
extern float yaw_offset;
extern float yaw_offset_tmp;

enum _position_mode
{
	libertine,
	locate,
	track,
};

extern enum _position_mode position_mode;


//遥控水平定位通道判断既高度模式判断
void position_mode_judge(void);

//与水平定位相关的事务处理--浪子、定位、循迹
void position_mode_handle(void);


//定点：单回路pid
extern struct _pid locate_single_loop_pid_x;
extern struct _pid locate_single_loop_pid_y;

void locate_single_loop_pid_control(float coordinate_x, float coordinate_y);
//void locate_cascade_control(float coordinate_x, float coordinate_y);

//循迹情况下输出限幅
void track_output_limit(float max, float *pitch);


//定点稳态判断
//
//@brief :	连续n次判断四轴相对于圆心的坐标current_position是否在稳态范围内
//
//@param :	1.四轴相对于圆心的坐标
//			2.四轴的期望坐标
//		 	3.判断的次数
//		 	4.稳态区间
//
//@retval:	1--已达到稳态
//			0--还没达到稳态
//
//@note	 :
//
u8 stady_state_judge(struct _position_float current_position, struct _position_float expect_position, u16 n_times, u16 steady_threshold);


#endif

