#ifndef _APP_H_
#define _APP_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"

extern float Battery_voltage;

typedef struct 
{
	u8 MPU6050_State;
	u8 HMC5883_State;
	u8 NRF2401_State;
}State;

void MCO_INIT(void);
void IAC_Init(void);
void Sensor_Init(void);
void State_Display(void);
void BATTDispaly(void);

#endif /* __APP_H */



