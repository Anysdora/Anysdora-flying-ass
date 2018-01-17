#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

//
//不要加入 #include "include.h", 造成错误
//
//

//路况状态机
typedef enum _road_state_machine
{
	road_state_none,
	one_circle,
	circle_connect_line,
	straight_line,
	line_connect_circle_connect_line,//4
	line_connect_circle,
	two_stripes,
	one_stripes,
	
	juxingjiao,//8
	Three_line_cricle,//9
	
	zone_c,
	zone_d,
	zone_e,
	zone_f,
	
}road_state_machine_type;

extern road_state_machine_type road_state_machine;
extern road_state_machine_type road_state_machine_last;



/***********************************************************************************************/



//题目要求模式状态机
typedef enum _mode_machine
{
	mode_none,	
	gesture_turning,
	force_to_auto_land,
	wait_setheight_spin_land,
	wait_setheight_setpoint_spin_track_setpoint_land,
	set_height_turning,
	set_point_turning,
	
	A_to_B_fire_B_to_A,		//7
	
	fly_on_rectangle,
	
	A_to_B,
	
	FIND_LCUK,
	
	B_to_A,
	
	set_height_,
	
	A_C_B,   							//A to C to B
	
}mode_machine_type;

extern mode_machine_type mode_machine;
extern mode_machine_type mode_machine_last;


#endif
