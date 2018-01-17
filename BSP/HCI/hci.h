#ifndef _HCI_H_
#define _HCI_H_

#include "include.h"


enum _level
{
	level_None = 1,
	level_zero,
	level_first,
	level_second,
	level_third,
	level_fourth,
	level_guard,
};

enum _level_first_item
{
	_level_first_None = 1,
	_level_first_Image,
	_level_first_Feedforward,
	_level_first_MachineChange,
	_level_first_guard,
};

enum _level_second_Feedforward
{
//	moto_None = 1,
	moto_0 = 1,
	moto_1,
	moto_2,
	moto_3,
	
	feed_pitch_offset,
	feed_roll_offset,
	
	test_start,
	moto_guard,
};
enum _level_second_MachineChange
{
	machine_None = 1,
	machine_A_B,
	machine_A_B_A,
	machine_RECT,
	machine_LUCK,
	machine_guard,
};

struct _level_second_item
{
	enum _level_second_Feedforward Feedforward_item;
	enum _level_second_MachineChange MachineChange_item;
};


enum _level_third_A 
{
	third_A_None = 1,
	third_A_height,

	third_A_start,
	third_A_guard,
};

enum _level_third_A_B 
{
	third_A_B_None = 1,
	third_A_B_height,

	third_A_B_start,
	third_A_B_guard,
};

enum _level_third_RECT 
{
	third_RECT_None = 1,
	third_RECT_height,

	third_RECT_start,
	third_RECT_guard,
};
enum _level_third_LUCK 
{
	third_LUCK_None = 1,
	third_LUCK_height,

	third_LUCK_start,
	third_LUCK_guard,
};


struct _level_third_item
{
	enum _level_third_A _level_third_A_item;				//ABA		
	enum _level_third_A_B _level_third_A_B_item;			
	enum _level_third_RECT _level_third_RECT_item;
	enum _level_third_LUCK _level_third_LUCK_item;

};

struct _hci
{
	enum _level current_level;
	enum _level last_level;
	
	enum _level_first_item current_first_level_item;

	struct _level_second_item current_second_level_item;
	
	struct _level_third_item current_third_level_item;
};



struct _delivery_param
{
	float expect_height_start_circle;
//	float expect_height_end_circle;
//	float fly_time;
//	float expect_angle_start_circle;
//	float expect_angle_end_circle;
};

extern struct _delivery_param delivery_param_to_control;





void hci(void);


#endif


