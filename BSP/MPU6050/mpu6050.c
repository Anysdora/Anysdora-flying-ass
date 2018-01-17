#include "MPU6050.h"
#include "include.h"
#define UART_Put_Char(x)  UART1_Put_Char(x)

struct _sensor sensor;	
u8 mpu6050_buffer[14];					//iic读取后存放数据 	

u8 ACC_OFFSET_OK = 0;

extern vs16 Moto_duty[4];

//**************************************
//初始化MPU6050
//**************************************
u8 InitMPU6050(void)
{
	u8 date;
	date  = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	//解除休眠状态
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	date += Single_Write(MPU6050_ADDRESS, cONFIG, 0x03);         //低通滤波
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x10);    //陀螺仪量程 +-1000
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);   //加速度量程 +-4G
	date += Single_Write(MPU6050_ADDRESS, BYPASS_CONFIG, 0x02);   //IIC旁路模式，可通过IIC访问HM5883
	return date;
}
//**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
//******************************************************************************
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, 0x3B);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, 0x3C);
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, 0x3D);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, 0x3E);
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, 0x3F);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, 0x40);
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, 0x43);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, 0x44);
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, 0x45);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, 0x46);
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, 0x47);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, 0x48);
}

//当pitch ，roll均稳定在0.02°以内时，校正完成
//校正完成返回1， 未完成返回0。
static u8 calibration_detection(float pitch, float roll)
{
	static float _pitch[30] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	static float _roll[30] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	static char cursor = 0;
	
	_pitch[cursor] = pitch;
	_roll[cursor] = roll;
	
	cursor++;
	if(30 == cursor)				//20个数据
	{
		cursor = 0;
	}
	
	for(int i = 0; i < 30; i++)
	{
		if(_pitch[i] > 0.04 || _pitch[i] < -0.04)
		{
			return 0;
		}
		if(_roll[i] > 0.04 || _roll[i] < -0.04)
		{
			return 0;
		}
	}
	
	return 1;
}
/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	MPU6050_Read();
	
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
  
	sensor.gyro.radian.x = sensor.gyro.origin.x * Gyro_Gr - sensor.gyro.quiet.x * Gyro_Gr;
	sensor.gyro.radian.y = sensor.gyro.origin.y * Gyro_Gr - sensor.gyro.quiet.y * Gyro_Gr;
	sensor.gyro.radian.z = sensor.gyro.origin.z * Gyro_Gr - sensor.gyro.quiet.z * Gyro_Gr;

	static u8 calibration_detection_flag = 0;

	//The calibration  of  gyro and  acc
	
	//①上电自动校正	②键盘一键校正
	//g_MatrixKey.calibration_flag.acc_gyro 初始化为1
	//ACC_OFFSET_OK 初始化为 0		, 		保证上电自动校正
	if(!ACC_OFFSET_OK && g_MatrixKey.calibration_flag.acc_gyro) 			
	{	
		LED2(ON);
		
		calibration_detection_flag = 0;
		
		//保证校正速度和准确度
		Kp = 2.0;
		Ki = 0.00;                   

		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;

		if(cnt_a==0)
		{
			sensor.acc.quiet.x = 0;
			sensor.acc.quiet.y = 0;
			sensor.acc.quiet.z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= sensor.acc.origin.x;
		tempay+= sensor.acc.origin.y;
		tempaz+= sensor.acc.origin.z;

		if(cnt_a==200)
		{
			sensor.acc.quiet.x = tempax/cnt_a;
			sensor.acc.quiet.y = tempay/cnt_a;
			sensor.acc.quiet.z = tempaz/cnt_a;
			cnt_a = 0;

//			可多次校正			
//			ACC_OFFSET_OK = 1;
			
			//在欧拉角yaw校正后g_MatrixKey.calibration会自动归零
			g_MatrixKey.calibration_flag.acc_gyro = 0;			
			
			LED2(OFF);

			return;
		}
		cnt_a++;		
	}
	
	//通过遥控校正传感器 
	if(!ACC_OFFSET_OK && Rc_Data.THROTTLE <= 1200 && Rc_Data.THROTTLE >= 900)
	{
		if(Rc_Data.ROLL <= 1200 && Rc_Data.PITCH >= 1750)
		{	
			LED2(ON);
			
			//euler_yaw 开始校正
			g_MatrixKey.calibration_flag.euler_yaw = 1;
			
			calibration_detection_flag = 0;
			
			//保证校正速度和准确度
			Kp = 2.0;
			Ki = 0.003;                   
						
			static int32_t	tempax=0,tempay=0,tempaz=0;
			static uint8_t cnt_a=0;

			if(cnt_a==0)
			{
				sensor.acc.quiet.x = 0;
				sensor.acc.quiet.y = 0;
				sensor.acc.quiet.z = 0;
				tempax = 0;
				tempay = 0;
				tempaz = 0;
				cnt_a = 1;
				return;
			}
			tempax+= sensor.acc.origin.x;
			tempay+= sensor.acc.origin.y;
			tempaz+= sensor.acc.origin.z;

			if(cnt_a==200)
			{
				sensor.acc.quiet.x = tempax/cnt_a;
				sensor.acc.quiet.y = tempay/cnt_a;
				sensor.acc.quiet.z = tempaz/cnt_a;
				cnt_a = 0;

//				只可校正1次		
				ACC_OFFSET_OK = 1;
				
				//欧拉角校正IMU.c文件euleryaw_correction();
//				g_MatrixKey.calibration_flag.euler_yaw = 1;
				LED2(OFF);
			}
			cnt_a++;				
		}
	}
	
	if(calibration_detection_flag == 0)
	{
		//保证飞控不受运动加速噪声的影响
		if(1 == calibration_detection(angle.pitch, angle.roll))
		{
			calibration_detection_flag = 1;
			
			Kp = 0.5;
			Ki = 0;
		}
	}
}

/**************************实现函数********************************************
//陀螺仪零点校准
*******************************************************************************/
void Gyro_OFFEST(void)
{
	uint16_t cnt_g=0;
	int32_t tempgx=100,tempgy=100,tempgz=100;
	int16_t gx_last=0,gy_last=0,gz_last=0;
	
	sensor.gyro.quiet.x=0;
	sensor.gyro.quiet.y=0;
	sensor.gyro.quiet.z=0;

	while(tempgx>=100 || tempgy>=100 || tempgz>=100)	//此循环是确保四轴处于完全静止状态
	{
		tempgx=0;tempgy=0;tempgz=0;cnt_g=400;
		while(cnt_g--)
		{
			MPU6050_Read();

			sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
			sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
			sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);

			tempgx += FL_ABS(sensor.gyro.origin.x - gx_last);
			tempgy += FL_ABS(sensor.gyro.origin.y - gy_last);
			tempgz += FL_ABS(sensor.gyro.origin.z - gz_last);

			gx_last = sensor.gyro.origin.x;
			gy_last = sensor.gyro.origin.y;
			gz_last = sensor.gyro.origin.z;
		}	 
	}
	
	tempgx=0;tempgy=0;tempgz=0;cnt_g=2000;

	while(cnt_g--)	 //此循环进行陀螺仪标定，前提是四轴已经处于完全静止状态
	{
		MPU6050_Read();

		sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
		sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
		sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);

		tempgx += sensor.gyro.origin.x;
		tempgy += sensor.gyro.origin.y;
		tempgz += sensor.gyro.origin.z;
	}

	sensor.gyro.quiet.x = tempgx/2000;
	sensor.gyro.quiet.y = tempgy/2000;
	sensor.gyro.quiet.z = tempgz/2000;	
}



