#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "include.h"

//上位机发送过来的数据 接收缓冲区
extern u8 USART_RX_BUF[50];	//9*2+4+1 == 23

//接收缓冲区 游标
extern u8 USART_RX_BUF_point;



extern struct _pid g_trans_data;

enum _error_report
{
	disarm_1 = 1,
	disarm_2,
	disarm_3,
	disarm_4,
	disarm_5,
	disarm_6,
};

extern enum _error_report error_report;

//向上位机报告数据

extern float test_report_data;
extern s16 test_report_data_s16;
extern s16 test_report_data_s16_1;
extern s16 test_report_data_s16_2;


void UART2_ReportIMU(void);


#endif	//_COMMUNICATION_H_


