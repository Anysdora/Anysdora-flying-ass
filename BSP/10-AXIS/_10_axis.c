#include "include.h"
#include "_10_axis.h"


struct _sensor sensor;

struct _angle angle;


void angle_calculation(signed long *quat, struct _angle *angle)
{
	float q30 = 1073741824.0f;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

	q0 = quat[0] / q30;
	q1 = quat[1] / q30;
	q2 = quat[2] / q30;
	q3 = quat[3] / q30;


	angle->roll = asin(-2 * q1 * q3 + 2 * q0* q2)* RtA;
	angle->pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA;
	angle->yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * RtA;
}


void sensor_gyro_calculation(signed short *gyro, struct _trans *sensor_gyro)
{
	sensor_gyro->angle.x = gyro[0] * Gyro_G;
	sensor_gyro->angle.y = gyro[1] * Gyro_G;
	sensor_gyro->angle.z = gyro[2] * Gyro_G;
}


void sensor_accel_calculation(signed short *accel, struct _trans *sensor_accel)
{
	sensor_accel->origin.x = accel[0];
	sensor_accel->origin.y = accel[1];
	sensor_accel->origin.z = accel[2];
}
