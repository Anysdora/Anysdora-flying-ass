#include "ano_tc.h"
#include "string.h"
#include "include.h"

//飞控向上位机发送姿态数据rol, pit, yaw, CSB, PRS
void Data_Send_Status(float Att_Angle_rol,float Att_Angle_pit,float Att_Angle_yaw,
					   float ALT_CSB, float ALT_PRS)
{
	u8 _cnt=0;
	s16 _temp;
	s32 _temp2;
	u8 sum = 0;
	u8 i;
	u8	data_to_send[50]={0};//准备发送的数据包

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Att_Angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (int)(Att_Angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (int)(Att_Angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (int)(ALT_CSB*1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp2 = (int)(ALT_PRS*100);
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
			
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	Uart2_Put_Buf(data_to_send,_cnt);
}

//飞控向上位机发送传感器数据
void Data_Send_Senser(int16_t Acc_X, int16_t Acc_Y,int16_t Acc_Z,
					  int16_t Gyr_X, int16_t Gyr_Y,int16_t Gyr_Z,
					  int16_t Mag_X, int16_t Mag_Y,int16_t Mag_Z)
{
	u8 _cnt=0;
	u8	data_to_send[50]={0};//准备发送的数据包

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Acc_X);  //高8位
	data_to_send[_cnt++]=BYTE0(Acc_X);  //低8位
	data_to_send[_cnt++]=BYTE1(Acc_Y);
	data_to_send[_cnt++]=BYTE0(Acc_Y);
	data_to_send[_cnt++]=BYTE1(Acc_Z);
	data_to_send[_cnt++]=BYTE0(Acc_Z);
	
	data_to_send[_cnt++]=BYTE1(Gyr_X);
	data_to_send[_cnt++]=BYTE0(Gyr_X);
	data_to_send[_cnt++]=BYTE1(Gyr_Y);
	data_to_send[_cnt++]=BYTE0(Gyr_Y);
	data_to_send[_cnt++]=BYTE1(Gyr_Z);
	data_to_send[_cnt++]=BYTE0(Gyr_Z);
	
	data_to_send[_cnt++]=BYTE1(Mag_X);
	data_to_send[_cnt++]=BYTE0(Mag_X);
	data_to_send[_cnt++]=BYTE1(Mag_Y);
	data_to_send[_cnt++]=BYTE0(Mag_Y);
	data_to_send[_cnt++]=BYTE1(Mag_Z);
	data_to_send[_cnt++]=BYTE0(Mag_Z);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	Uart2_Put_Buf(data_to_send,_cnt);
}

//飞控向上位机发送电机数据 Moto
//PWM范围：0~1000
void Data_Send_MotoPWM(int16_t Moto_PWM_1, int16_t Moto_PWM_2,
					   int16_t Moto_PWM_3, int16_t Moto_PWM_4)
{
	u8 _cnt=0;
	u8	data_to_send[50]={0};//准备发送的数据包

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(Moto_PWM_1);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_1);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_2);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_2);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_3);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_3);
	data_to_send[_cnt++]=BYTE1(Moto_PWM_4);
	data_to_send[_cnt++]=BYTE0(Moto_PWM_4);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Uart2_Put_Buf(data_to_send,_cnt);
}

//飞控向上位机发送遥控数据
void Data_Send_RCData(int16_t Rc_Throttle, int16_t Rc_Yaw,
					  int16_t Rc_Roll, int16_t Rc_Pitch)
{
	u8 _cnt=0;
	u8	data_to_send[50]={0};//准备发送的数据包
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(Rc_Throttle);
	data_to_send[_cnt++]=BYTE0(Rc_Throttle);
	data_to_send[_cnt++]=BYTE1(Rc_Yaw);
	data_to_send[_cnt++]=BYTE0(Rc_Yaw);
	data_to_send[_cnt++]=BYTE1(Rc_Roll);
	data_to_send[_cnt++]=BYTE0(Rc_Roll);
	data_to_send[_cnt++]=BYTE1(Rc_Pitch);
	data_to_send[_cnt++]=BYTE0(Rc_Pitch);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Uart2_Put_Buf(data_to_send,_cnt);
}

//整理向上位机发送自定义的数据包1 
void Data_Send_DIY1(s16 _temp1,s16 _temp2,s16 _temp3,s32 _temp4)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	u8	data_to_send[50]={0};//准备发送的数据包

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;	   //上位机中 1帧		 F1~F10
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);

	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	data_to_send[_cnt++]=BYTE1(_temp3);
	data_to_send[_cnt++]=BYTE0(_temp3);
	
	data_to_send[_cnt++]=BYTE3(_temp4);	  //_temp4为s32型占4个字节
	data_to_send[_cnt++]=BYTE2(_temp4);
	data_to_send[_cnt++]=BYTE1(_temp4);
	data_to_send[_cnt++]=BYTE0(_temp4);	  //最多支持30个不同类型数据
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	Uart2_Put_Buf(data_to_send,_cnt);
}

//整理向上位机发送自定义的数据包2 
void Data_Send_DIY2(u8 dz,s16 _temp1,s16 _temp2,s16 _temp3,s32 _temp4)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	u8	data_to_send[50]={0};//准备发送的数据包

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=dz;	   //上位机中 1帧		 F1~F10
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);

	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	data_to_send[_cnt++]=BYTE1(_temp3);
	data_to_send[_cnt++]=BYTE0(_temp3);
	
	data_to_send[_cnt++]=BYTE3(_temp4);	  //_temp4为s32型占4个字节
	data_to_send[_cnt++]=BYTE2(_temp4);
	data_to_send[_cnt++]=BYTE1(_temp4);
	data_to_send[_cnt++]=BYTE0(_temp4);	  //最多支持30个不同类型数据
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	Uart2_Put_Buf(data_to_send,_cnt);
}

//飞控向上位机发送接收数据校验
void Data_Send_Check(u16 check)
{	u8 sum = 0;
	u8 i=0;
	u8	data_to_send[50]={0};//准备发送的数据包

	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);

	for(i=0;i<7;i++)
		sum += data_to_send[i];
	
	data_to_send[7]=sum;

	Uart2_Put_Buf(data_to_send,8);
}

/////////////////////////////////////////////////////////////////////////////////////
//communication.h文件中
//飞控接收上位机发过来的数据包，存在USART_RX_BUF数组里
//extern u8 USART_RX_BUF[23];	//(9*2+4+1)*2 == 46
/////////////////////////////////////////////////////////////////////////////////////

//无用数据，补位专业
_pid scapegoat_1, scapegoat_2, scapegoat_3;

//解析上位机发送过来的数据
void Data_Receive_Pid1(_pid *PID_ROL,_pid *PID_PIT,_pid *PID_YAW)
{	
	u8 sum = 0;
	u8 i = 0,j = 0;
	u8 num = 23;    //一组数据有23位  9*2+4+1 == 23
	u8 *data_buf;   //u8 unsigned char占一个字节 

	data_buf=USART_RX_BUF; //数据包首地址传给data_buf
	for(j=0;j<23;j++)
	{
		if(*(data_buf+j)==0xAA && *(data_buf+j+1)==0xAF)//判断帧头	
		{
		//	*(data_buf+j)=0x00;
			data_buf=data_buf+j;	
			break;
		}		
	}

	//PID1
	if(*(data_buf+2)==0X10)								//PID1 数据帧1 0X10
	{
		for(i=0;i<(num-1);i++)							//数据包长度num=5+X,	  此处，X=9*2
			sum += *(data_buf+i);
		if(!(sum==*(data_buf+num-1)))		return;		//校验，判断sum
			
		PID_ROL->kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;	//上位机
		PID_ROL->ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10000;
		PID_ROL->kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;

		PID_PIT->kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/1000;
		PID_PIT->ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/10000;
		PID_PIT->kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;

		PID_YAW->kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/1000;
		PID_YAW->ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/10000;
		PID_YAW->kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		
		Data_Send_Check(sum);
		
		//清空缓冲区，同时将游标置零
		USART_RX_BUF_point = 0;						
		memset(USART_RX_BUF, 0, 23);
	}
}

void Data_Receive_Pid2(_pid *PID_ROL,_pid *PID_PIT,_pid *PID_YAW)
{	
	u8 sum = 0;
	u8 i = 0,j = 0;
	u8 num = 23;
	u8 *data_buf;

	data_buf=USART_RX_BUF;
	for(j=0;j<23;j++)
	{
		if(*(data_buf+j)==0xAA && *(data_buf+j+1)==0xAF)//判断帧头	
		{
			//*(data_buf+j)=0x00;
			data_buf=data_buf+j;	
			break;
		}		
	}

	//PID2
	if(*(data_buf+2)==0X11)								//PID2 数据帧2 0X11
	{
		for(i=0;i<(num-1);i++)							//数据包长度num=5+X,	  此处，X=9*2
			sum += *(data_buf+i);
		if(!(sum==*(data_buf+num-1)))		return;		//校验，判断sum
			
		PID_ROL->kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;	//上位机
		PID_ROL->ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10000;
		PID_ROL->kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;

		PID_PIT->kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
		PID_PIT->ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/10000;
		PID_PIT->kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;

		PID_YAW->kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
		PID_YAW->ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/10000;
		PID_YAW->kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		
		Data_Send_Check(sum);

		//清空缓冲区，同时将游标置零		
		USART_RX_BUF_point = 0;						
		memset(USART_RX_BUF, 0, 23);		
	}
}

void Data_Receive_lock(u8 *armed)
{
	u8 j = 0;
	u8 *data_buf;

	data_buf=USART_RX_BUF;
	for(j=0;j<23;j++)
	{
		if(*(data_buf+j)==0xAA && *(data_buf+j+1)==0xAF)//判断帧头	
		{
			data_buf=data_buf+j;	
			break;
		}		
	}
	
	//
	//PID2
	if(*(data_buf+2) == 0X01)								//传感器校正 数据帧2 0X01
	{
		if(*(data_buf+4)==0X01 && *(data_buf+5)==0X5C)
		{
			//加速度校正
			

			//解锁**********************************************************************************************/
			*armed = 1;
		}
		if(*(data_buf+4)==0X02 && *(data_buf+5)==0X5D)
		{
			//陀螺仪校正


			//上锁**********************************************************************************************/
			*armed = 0;
		}
		if(*(data_buf+4)==0X04 && *(data_buf+5)==0X5F)
		{
			//罗盘校正
				mode_3_flow_steps=mode_step_3;    //定高模式自动降落
				mode_4_flow_steps=mode_step_3;    //定点模式自动降落
				mode_A_to_B_steps=mode_step_10;    //ATB模式自动降落
				mode_A_to_B_fire_B_to_A_flow_steps=mode_step_10;    
				mode_fly_on_rectangle_flow_steps=mode_step_17;
				mode_B_to_A_steps=mode_step_7;
				mode_A_C_B_steps=mode_step_11;
				mode_FIND_LCUK_steps=mode_step_10;
			
//			mode_machine =fly_on_rectangle;
//			mode_machine = A_to_B_fire_B_to_A;
//			mode_machine = wait_setheight_setpoint_spin_track_setpoint_land;
//			mode_machine = set_point_turning;
//			mode_machine = gesture_turning;
//			mode_machine = wait_setheight_spin_land;
//			mode_machine = set_height_turning;			
			
		}
		if(*(data_buf+4)==0X05 && *(data_buf+5)==0X60)
		{
			//气压计归零
			
			
//			mode_A_to_B_fire_B_to_A_flow_steps = mode_step_10;
//			mode_fly_on_rectangle_flow_steps =  mode_step_11;
//			mode_A_to_B_steps = mode_step_10;			
//			
//			mode_FIND_LCUK_steps = mode_step_10;
//			mode_B_to_A_steps = mode_step_10;
			
//			mode_5_flow_steps = mode_step_7;			
//			mode_3_flow_steps = mode_step_2;
//			mode_4_flow_steps = mode_step_2;		
		}
							
		//清空缓冲区，同时将游标置零
		USART_RX_BUF_point = 0;						
		memset(USART_RX_BUF, 0, 23);	
	}	
}
	



