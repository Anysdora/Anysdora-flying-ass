#include"include.h"
#include"height.h"

//定高模式	默认手控
enum _height_mode height_mode;
enum _height_mode height_mode_last;


//定高内环滤波
#define KALMAN_Q        0.2
#define KALMAN_R        6.0000
/*           卡尔曼对三个轴加速度进行滤波处理           */
static double KalmanFilter_speed(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;
	static double x_last;
	double x_mid = x_last;
	double x_now;
	static double p_last;
	double p_mid ;
	double p_now;
	double kg;        

	x_mid=x_last; 			//x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:上一次的最优预测值，x_mid:现在从上一次来的预测值
	p_mid=p_last+Q; 		//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声   //p_mid：本次从上一次来的协方差，p_last:上次的协方差
	kg=p_mid/(p_mid+R); 	//kg为kalman filter，R为噪声     	//kg卡尔曼增益   ResrcData：仪器的测量值
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值    		//本次的最优预测值
		
	p_now=(1-kg)*p_mid;		//最优值对应的covariance        	//本次的协方差
	p_last = p_now; 		//更新covariance值               	//以下两句完成递归
	x_last = x_now; 		//更新系统状态值
	return x_now;   		//返回最优估计值          
 }

/**************************************************************************************************************/
//功  能：起飞、降落 期望值设定曲线，分为快速升降阶段和缓慢升降阶段
//参  数：step_high_time：  快速上升时间为，单位 n * 40 （ms）
//		：step_low_time :   慢速上升时间为，单位 n * 40 （ms）
//		: zero_height   : 	期望值改变瞬间高度
//		: expect_height :	最终期望高度
//说  明: 此函数在同一种期望高度下只能调用一次。
/**************************************************************************************************************/
static float wave_expect_height(float expect_height, float zero_height, u16 step_high_time, u16 step_low_time)
{
	//当last_height == 0时, 如果expect_height有一次为0, 则会出现bug.
	static float last_height = -1314;

	static s16 high_time_count = 1;							//high_speed
	static s16 low_time_count = 1;							//low_speed
	
	float ret_ept_hgt = zero_height;

	if(last_height != expect_height)
	{		
		last_height = expect_height;
		
		high_time_count = 1;
		low_time_count = 1;
	}	
	
	float step1_distance = (expect_height - zero_height) * 0.75;
	float step2_distance = (expect_height - zero_height) * 0.25;
	float high_time_count_max = step_high_time;
	float low_time_count_max =  step_low_time;

	if(high_time_count <= high_time_count_max)
	{
		ret_ept_hgt += step1_distance / step_high_time * (high_time_count++);
	}
	else if(low_time_count <= low_time_count_max)
	{
		ret_ept_hgt += step1_distance + step2_distance / step_low_time * (low_time_count++);
	}
	else
	{
		ret_ept_hgt = expect_height;
	}

	return ret_ept_hgt;
}

/************************************************************************/
/*void auto_land_control(void)											*/
/*控制策略：不完全微分控制	--  pid_out低通滤波							*/
/*输入：			                               						*/
/*输出：                                             					*/
/************************************************************************/
#define auto_land_alfa 0.1
struct _pid auto_land_pid;

struct _height_grab auto_land_height_grab = {0, 0};

float auto_land_last_height;

void auto_land_control(void)
{
	float error;

	float auto_land_pid_out;
		
	/********************************************************************************
	//当第一次抓取油门时更新static变量数据, 防止历史遗留数据对当前控制产生影响
	//auto_land_throttle_grab.flag在油门抓取后置 1
	if(auto_land_throttle_grab.flag == 0)
	{
		auto_land_pid.pid_out = 0;
	}
	********************************************************************************/	

	
	//油门抓取	
	if(auto_land_throttle_grab.flag == 0)
	{
		auto_land_throttle_grab.flag = 1;
		
		//由定高走向自动降落
//		auto_land_throttle_grab.current_throttle = set_height_throttle_grab.current_throttle + ctrl.height.core.pid_out;
	}
		
	//高度抓取
	if(auto_land_height_grab.flag == 0)	
	{	
		auto_land_height_grab.flag = 1;
	
		auto_land_height_grab.current_height = g_height;
	}

	//期望值曲线 必须置于自动上锁之前
	float expect_height = wave_expect_height(-30 , auto_land_height_grab.current_height, 100, 100);
/*************************************************************************************************/
	test_report_data_s16 = expect_height;
/*************************************************************************************************/
	
	//自动降落模式下，高度低于10cm自动上锁。
	if(g_height <= 10)
	{	
		ARMED = 0;
		
		return ;
	}
	
	//
	auto_land_pid_set();

	error = expect_height - g_height;
	
	auto_land_pid.kp_out = auto_land_pid.kp * error;
	
	auto_land_pid.increment += error;
	if(auto_land_pid.increment > auto_land_pid.increment_max)
	{
		auto_land_pid.increment  = auto_land_pid.increment_max;
	}
	else if(auto_land_pid.increment < -auto_land_pid.increment_max)
	{
		auto_land_pid.increment = -auto_land_pid.increment_max;
	}
	auto_land_pid.ki_out = auto_land_pid.ki * auto_land_pid.increment;
	
	auto_land_pid.kd_out = auto_land_pid.kd * (auto_land_last_height - g_height);
	
	auto_land_last_height = g_height;
	
	auto_land_pid_out = auto_land_pid.kp_out 
						  + auto_land_pid.ki_out
						  + auto_land_pid.kd_out;
	
	//一阶低通滤波
	auto_land_pid_out = auto_land_alfa * auto_land_pid_out  + (1 - auto_land_alfa) * auto_land_pid.pid_out;			  
						  
	auto_land_pid.pid_out = auto_land_pid_out;
}

//阶梯型期望值设定
static float step_expect_height(float expect_height, float current_height)
{
	float return_expect_height;
		
	if(current_height < expect_height)
	{
		switch((int)(current_height / 20))
		{
			case 0:		return_expect_height = 20;	break;
			case 1:		return_expect_height = 40;	break;
			case 2:		return_expect_height = 60;	break;
			case 3:		return_expect_height = 80;	break;
			case 4:		return_expect_height = 100;	break;
			case 5:		return_expect_height = 120;	break;
			case 6:		return_expect_height = 140;	break;
			case 7:		return_expect_height = 160;	break;
			case 8:		return_expect_height = 180;	break;
			case 9:		return_expect_height = 200;	break;
			case 10:	return_expect_height = 220;	break;
			case 11:	return_expect_height = 240;	break;
			case 12:	return_expect_height = 260;	break;
			case 13:	return_expect_height = 280;	break;
			case 14:	return_expect_height = 300;	break;
			default:	return_expect_height = 0;	break;	
		}
	}
	else if(current_height > expect_height)
	{
		switch((int)(current_height / 20))
		{
			case 1:		return_expect_height = 20;	break;
			case 2:		return_expect_height = 40;	break;
			case 3:		return_expect_height = 60;	break;
			case 4:		return_expect_height = 80;	break;
			case 5:		return_expect_height = 100;	break;
			case 6:		return_expect_height = 120;	break;
			case 7:		return_expect_height = 140;	break;
			case 8:		return_expect_height = 160;	break;
			case 9:		return_expect_height = 180;	break;
			case 10:	return_expect_height = 200;	break;
			case 11:	return_expect_height = 220;	break;
			case 12:	return_expect_height = 240;	break;
			case 13:	return_expect_height = 260;	break;
			case 14:	return_expect_height = 280;	break;
			case 15:	return_expect_height = 300;	break;
			default:	return_expect_height = 0;	break;	
		}
	}
	else
	{
		return_expect_height = expect_height;
	}
	
	if((expect_height - return_expect_height > -19.99) && (expect_height - return_expect_height < 0))
	{
		return expect_height;
	}
	
	return return_expect_height;

}

static u8 set_height_start_control(void)   //油门基准设置
{
	static u8 flag = 0;

/********************************************************************************/
	//只抓取一次
	if(set_height_throttle_grab.flag == 0)
	{
		set_height_throttle_grab.flag = 1;
		
		set_height_throttle_grab.current_throttle = 1200;    //当前油门1200
		flag = 1;                                            //flag置1
	}
/********************************************************************************/

	if(flag)
	{
		if(g_height < height_base+5)               //#define height_base 4.0f
		{
			
			if(set_height_throttle_grab.current_throttle <1250 && set_height_throttle_grab.current_throttle >1000)
			{
				set_height_throttle_grab.current_throttle++;
			}
			else 
			{
				set_height_throttle_grab.current_throttle += 5.7;	
			}
			return 0;        					           							//基准设置完成
		}
		else
		{
			//三叶桨
			set_height_throttle_grab.current_throttle += 16;
			flag = 0;
		}
	}
	return 1;
}

//定高期望值
float set_height_control_expect_height = 0;


//曲线型期望值设定
struct _height_grab set_height_height_grab = {0, 0};

static void set_height_cascade_control(float expect_height)
{
	static u8 control_rate;
	
	float error;
	static float shell_last_height;
	
	if(!set_height_start_control())    //油门基准设置未完成
	{
		return;
	}

	//期望值曲线
	expect_height = wave_expect_height(expect_height, set_height_height_grab.current_height, 100, 100);
/*************************************************************************************************/
	test_report_data_s16 = expect_height;
/*************************************************************************************************/
		
	if(control_rate == 2)
	{	
		error = expect_height - g_height;

		ctrl.height.shell.increment += error;
//		//积分限幅
		if(ctrl.height.shell.increment > ctrl.height.shell.increment_max)
			ctrl.height.shell.increment = ctrl.height.shell.increment_max;
		else if(ctrl.height.shell.increment < -ctrl.height.shell.increment_max)
			ctrl.height.shell.increment = -ctrl.height.shell.increment_max;
		
		ctrl.height.shell.pid_out = ctrl.height.shell.kp * error 
								  + ctrl.height.shell.ki * ctrl.height.shell.increment 
								  + ctrl.height.shell.kd * (shell_last_height - g_height);//微分先行			//微分先行
		shell_last_height = g_height;

		control_rate = 0;
	}
	control_rate++;
	
	float height_core_error;
	static float core_last_error;
	float vertical_speed;
	static float core_last_height;
	
	vertical_speed = (g_height - core_last_height) / 0.04;							// 	cm/s
	core_last_height = g_height;
	
	height_core_error = ctrl.height.shell.pid_out - vertical_speed;
	
	ctrl.height.core.kp_out = ctrl.height.core.kp * height_core_error;

	ctrl.height.core.increment += height_core_error;   		
	if(ctrl.height.core.increment > ctrl.height.core.increment_max)
			ctrl.height.core.increment = ctrl.height.core.increment_max;
	else if(ctrl.height.core.increment < -ctrl.height.core.increment_max)
			ctrl.height.core.increment = -ctrl.height.core.increment_max;
	ctrl.height.core.ki_out = ctrl.height.core.ki * ctrl.height.core.increment; 
	
	//微分先行
	ctrl.height.core.kd_out = ctrl.height.core.kd * (height_core_error - core_last_error);
	
	core_last_error = height_core_error;
	
	ctrl.height.core.pid_out = ctrl.height.core.kp_out 
							 + ctrl.height.core.ki_out 
							 + ctrl.height.core.kd_out;	
}


void set_height_cascade_control_2(float expect_height, u16 high_speed_time, u16 low_speed_time)//定高PID计算
{
	float error; 
	static float shell_last_height;     //外环之前高度
	static u8 control_rate = 0 ;
	
	float height_core_error;              
	static float core_last_speed;
	float vertical_speed;               //垂直速度
	static float core_last_height;      //内环之前高度
	
	if(g_height == 300)
	{
		return;
	}
	if(fabs(core_last_height-g_height)>15)//优化1.0
	{
		g_height=core_last_height;
	}
	//对于特殊数据不做处理

	
/********************************************************************************/
	//当第一次抓取油门时更新static变量数据, 防止历史遗留数据对当前控制产生影响
	//set_height_throttle_grab.flag在set_height_start_control()中置 1
	if(set_height_throttle_grab.flag == 0)
	{
		shell_last_height = g_height;    
		core_last_speed = 1.1;
		core_last_height = g_height;
	}
/********************************************************************************/	
	
	if( !set_height_start_control() )    //油门基准设置未完成
	{
		return;
	}
	
	//高度抓取, 为设置期望值曲线做准备,
	//一条曲线只抓一次
	if(set_height_height_grab.flag == 0)
	{	
		set_height_height_grab.flag = 1;
	
		set_height_height_grab.current_height = g_height;
	}

	
	//期望值曲线
	expect_height = wave_expect_height(expect_height, set_height_height_grab.current_height, high_speed_time, low_speed_time);
/*************************************************************************************************/
	test_report_data_s16 = expect_height;
/*************************************************************************************************/
	
	//外环 -- 高度
	control_rate++;
	if(control_rate == 2)
	{
			control_rate = 0;
		error = expect_height - g_height;

		ctrl.height.shell.increment += error;

	//		//积分限幅
		if(ctrl.height.shell.increment > ctrl.height.shell.increment_max)
			ctrl.height.shell.increment = ctrl.height.shell.increment_max;
		else if(ctrl.height.shell.increment < -ctrl.height.shell.increment_max)
			ctrl.height.shell.increment = -ctrl.height.shell.increment_max;

		ctrl.height.shell.pid_out = ctrl.height.shell.kp * error 
									+ ctrl.height.shell.ki * ctrl.height.shell.increment 
									+ ctrl.height.shell.kd * (shell_last_height - g_height);//微分先行			//微分先行
		shell_last_height = g_height;
	
	}

	//内环 -- 速度
	vertical_speed = (g_height - core_last_height) ;							// 	(*25cm/s)
	
//	vertical_speed = KalmanFilter_speed(vertical_speed, KALMAN_Q, KALMAN_R);
//	
	//只在期望值附近使用低通滤波
//	if(g_height - expect_height < 20 && g_height - expect_height > -20)
//	{
//		vertical_speed = 0.95 * core_last_speed  + 0.05 * vertical_speed;
//	}
	
	core_last_height = g_height;		
	
	height_core_error = ctrl.height.shell.pid_out - vertical_speed;
	
	ctrl.height.core.kp_out = ctrl.height.core.kp * height_core_error;

	ctrl.height.core.increment += height_core_error;   		
	if(ctrl.height.core.increment > ctrl.height.core.increment_max)
			ctrl.height.core.increment = ctrl.height.core.increment_max;
	else if(ctrl.height.core.increment < -ctrl.height.core.increment_max)
			ctrl.height.core.increment = -ctrl.height.core.increment_max;
	
	
	if(error > 5 || error < -5)
	{

		ctrl.height.core.increment += 2.5 * height_core_error;   		
	}
	else
	{
		ctrl.height.core.increment += height_core_error; 
	}
	
	ctrl.height.core.ki_out = ctrl.height.core.ki * ctrl.height.core.increment; 
	
	//微分先行
	ctrl.height.core.kd_out = ctrl.height.core.kd * (core_last_speed - vertical_speed);
	core_last_speed = vertical_speed;
	
	ctrl.height.core.pid_out = ctrl.height.core.kp_out 
							 + ctrl.height.core.ki_out 
							 + ctrl.height.core.kd_out;	

}


//遥控定高通道判断既高度模式判断
void height_mode_judge(void)
{
	if(Rc_Data_T8_C3 < 1100 && Rc_Data_T8_C3 > 1080)				//0
	{		
		mode_machine = mode_none;
	}
	else if(Rc_Data_T8_C3 < 1500 && Rc_Data_T8_C3 > 1480)			//1
	{	
		//进入自动流程控制
		mode_machine = wait_setheight_spin_land;
	}
	else if(Rc_Data_T8_C3 < 1900 && Rc_Data_T8_C3 > 1880)			//2														//自动起飞 + 定高
	{		
		mode_machine = force_to_auto_land;
	}
	else															//遥控联通时无此选项
	{	
		//do nothing
	}
	
	if(!ARMED)
	{
		mode_machine = mode_none;
	}
}

//自动降落PID给定
void auto_land_pid_set(void)
{
	if(g_height > 100)
	{
		auto_land_pid.kd  = 4;
	}
	else if(g_height > 40)
	{
		auto_land_pid.kd = 2;
	}
	else
	{
//		auto_land_pid.kp = 0.8;
		auto_land_pid.kd  = 0;
	}
}

//height_variable_reset
//与定高有关变量初始化
void height_variable_reset()
{
	if(height_mode != set_height)
	{
		set_height_throttle_grab.flag = 0;
		
		ctrl.height.core.pid_out = 0;
		ctrl.height.shell.pid_out = 0;
	}
	
	if(height_mode != auto_land)
	{
		auto_land_throttle_grab.flag = 0;
		auto_land_height_grab.flag = 0;
		
		auto_land_pid.pid_out = 0;
		auto_land_pid.increment = 0;
		
		auto_land_last_height = g_height;
	}	
}
/************************************************************************/
/*void height_mode_handle(void)											*/
/*策略：处理各种与高度有关的控制										*/
/*关键问题：①抓取当前油门 ②积分、标志清零              				*/
/*输入：			                               						*/
/*输出：                                             					*/
/************************************************************************/
struct _throttle_grab auto_land_throttle_grab = {0, 0};
struct _throttle_grab set_height_throttle_grab = {0, 0};

void height_mode_handle(void)
{
	height_variable_reset();
	
	if(height_mode == manually)
	{
		//此函数用于更新内部static变量
		float expect = wave_expect_height(99, 100, 10, 10);
	}
	else if(height_mode == auto_land)
	{
//		//油门抓取	
//		if(auto_land_throttle_grab.flag == 0)
//		{
//			auto_land_throttle_grab.flag = 1;
//			
//			//由定高走向自动降落
//			auto_land_throttle_grab.current_throttle = set_height_throttle_grab.current_throttle;
//		}
		
//		//高度抓取
//		if(auto_land_height_grab.flag == 0)	
//		{	
//			auto_land_height_grab.flag = 1;
//		
//			auto_land_height_grab.current_height = g_height;
//		}

		
		//自动降落控制
//		auto_land_control();
	}
	else if(height_mode == set_height)
	{
		//油门抓取
//		if(set_height_throttle_grab.flag == 0)
//		{
//			set_height_throttle_grab.flag = 1;
//		}
		
		
		//定高串级pid
//		set_height_cascade_control_2(set_height_control_expect_height);
		
	}
	else
	{
		//do nothing
	}
	
//	height_mode_last = height_mode;
}

