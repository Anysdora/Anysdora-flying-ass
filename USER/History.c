////////////////////////////////////////////////////////////
static float wave_expect_height_2(float expect_height, float zero_height, enum _height_mode current_mode)
{
	static enum _height_mode last_mode = manually;

	static s16 high_time_count = 1;							//high_speed
	static s16 low_time_count = 1;							//low_speed
	
	float step1_distance = 30;
	
	float ret_ept_hgt = zero_height;

	if(last_mode != current_mode)
	{
		last_mode = current_mode;
		high_time_count = 1;
		low_time_count = 1;
	}	

	ret_ept_hgt += -high_time_count*1.2;
	
	if(ret_ept_hgt < step1_distance)
	{
		ret_ept_hgt += -low_time_count*3;
		
		if(ret_ept_hgt < expect_height)
		{
			return ret_ept_hgt;
		}
		
		low_time_count++;
	}
	
	high_time_count++;
	
	return ret_ept_hgt;
}
