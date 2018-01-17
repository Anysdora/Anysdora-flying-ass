#ifndef __INV_MPU_USER_H
#define __INV_MPU_USER_H

#include "include.h"

void inv_mpu_read(struct _angle *angle, struct _sensor *sensor);
int mpu_dmp_init(void);
void run_self_test(void);

#endif //__INV_MPU_USER_H

