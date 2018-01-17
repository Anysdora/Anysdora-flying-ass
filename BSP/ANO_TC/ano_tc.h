#ifndef __ANO_TC_H
#define __ANO_TC_H

#include "usart.h"
#include "delay.h"

#define BYTE3(_temp) 	(u8)(_temp>>24)
#define BYTE2(_temp) 	(u8)(_temp>>16)
#define BYTE1(_temp) 	(u8)(_temp>>8)
#define BYTE0(_temp) 	(u8)_temp

typedef struct _pid		_pid;

//无用数据，补位专业
extern _pid scapegoat_1, scapegoat_2, scapegoat_3;


//飞控向上位机发送姿态数据rol, pit, yaw, CSB, PRS
//	pit*100 的值的范围：-180*100 ~ 180*100
//	rol*100, yaw*100 的值的范围：-32767 ~ 32768
//	CSB*1000 的值的范围：-32767 ~ 32768 
//	PRS*100 的值的范围：-2^31 ~ 2^31-1	
void Data_Send_Status(float Att_Angle_rol,float Att_Angle_pit,float Att_Angle_yaw,
					   float ALT_CSB, float ALT_PRS);

//飞控向上位机发送传感器数据
void Data_Send_Senser(int16_t Acc_X, int16_t Acc_Y,int16_t Acc_Z,
					  int16_t Gyr_X, int16_t Gyr_Y,int16_t Gyr_Z,
					  int16_t Mag_X, int16_t Mag_Y,int16_t Mag_Z);

//飞控向上位机发送电机数据 Moto
//PWM范围：0~1000
void Data_Send_MotoPWM(int16_t Moto_PWM_1, int16_t Moto_PWM_2,
					   int16_t Moto_PWM_3, int16_t Moto_PWM_4);

//飞控向上位机发送遥控数据
void Data_Send_RCData(int16_t Rc_Throttle, int16_t Rc_Yaw,
					  int16_t Rc_Roll, int16_t Rc_Pitch);

//整理向上位机发送自定义的数据包1 
void Data_Send_DIY1(s16 _temp1,s16 _temp2,s16 _temp3,s32 _temp4);

//整理向上位机发送自定义的数据包2 
void Data_Send_DIY2(u8 dz,s16 _temp1,s16 _temp2,s16 _temp3,s32 _temp4);

void Data_Send_Check(u16 check);

//解析上位机发送过来的数据包 pid1
void Data_Receive_Pid1(struct _pid *PID_ROL,struct _pid *PID_PIT,struct _pid *PID_YAW);

//解析上位机发送过来的数据包 pid2
void Data_Receive_Pid2(struct _pid *PID_ROL,struct _pid *PID_PIT,struct _pid *PID_YAW);

//解析上位机发送过来的数据包 上锁解锁(上位机上对应的是传感器的校正)
void Data_Receive_lock(u8 *armed);

#endif
