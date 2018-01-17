#ifndef __IMU_H
#define	__IMU_H
#include "stm32f10x.h"
#include "include.h"

#define RtA 	57.324841		//  180/3.1415  角度制 转化为弧度制		
#define AtR    	0.0174533		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G 	0.03051756	//  1/32768/1000      陀螺仪量程为 +—1000			
#define Gyro_Gr	0.0005326   //  1/32768/1000/57.3 

#define halfT 0.00125f                 // 采样周期的一半  本程序 2.5MS 采集一次  所以 halfT是1.25MS

extern float Kp;
extern float Ki;

struct _angle
{
	float pitch;
	float roll;
	float yaw;
};

extern struct _angle angle;

//上电后欧拉角值稳定后给yaw校正
extern float yaw_offset;

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
//欧拉角yaw校正
void euleryaw_correction(void);

double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);

float SIN(float y);
float FL_ABS(float x);

#endif













