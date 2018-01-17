#ifndef _OLED_DISPLAY_H_
#define _OLED_DISPLAY_H_

#include "include.h"

void State_Display(void);

void BATTDispaly(void);

void State_Display(void);



extern float Battery_voltage;

void parameterDispaly(void);


enum _dispaly_data
{
	param,
	image,
};

extern enum _dispaly_data display_what;



void display_imgae(u8 *camera_image_buf);

#endif

