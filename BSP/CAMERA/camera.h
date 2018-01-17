#ifndef	_CAMERA_H_
#define _CAMERA_H_
#include "include.h" 


//摄像头模块传输过来的图像数据
//图像缓冲区指针

extern u8 camera_image_buf[603];

enum _new_imgae_flag 
{
	came,
	coming,
};

extern enum _new_imgae_flag new_image_flag;



//四轴飞行过程中的朝向
enum _direction
{
	forward,
	backward,
};

extern enum _direction fly_direction;

struct _position_s16
{
	s16 x;
	s16 y;
};


struct _position_float
{
	float x;
	float y;
};

extern struct _position_s16 g_position_circle_receive;
extern struct _position_float g_position_circle_adjust;

extern struct _position_s16 g_position_line_1_receive;
extern struct _position_float g_position_line_1_adjust;

extern struct _position_s16 g_position_line_2_receive;
extern struct _position_float g_position_line_2_adjust;


extern u8 USART_ov7620_rxbuf[1206];

extern u16 camera_RX_BUF_point;
extern float g_height_original;


enum _get_data_from_camera
{
	get_position_data,
	get_image,
};


//向摄像头模块发送获取图像指令或位置数据指令
void get_image_or_position_data(enum _get_data_from_camera cmd);


void camera_data_image_decode(u8 *ov7620_rxbuf, u16 *ov7620_rxbuf_cursor);


//位置数据合成已经校正
void position_data_decode(u8 *data_array, u16 *ov7620_rxbuf_cursor, struct _position_float *position_circle, struct _position_float *position_line_1, struct _position_float *position_line_2);


float get_angle_of_line_relative_to_circle(float circle_center_x, float circle_center_y, float line_endpoint_x, float line_endpoint_y);


struct _expect_yaw_grab
{
	u8 flag;
	float current_expect_yaw;
};

extern struct _expect_yaw_grab expect_yaw_grab;


float get_yaw_expect(float circle_center_x, float circle_center_y, float line_endpoint_x, float line_endpoint_y);

//yaw期望值曲线
u8 yaw_expect_wave(float yaw_expect_temp, float *yaw_off);


#endif

