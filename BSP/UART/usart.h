#ifndef __USART_H_
#define __USART_H_
#include "stm32f10x.h"
#include <stdio.h>


void PC_Debug_Show(u8 num,u16 sta);

int fputc(int ch, FILE *f);
void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...);


void usart1_config(void);
uint8_t UART1_Put_Char(unsigned char DataToSend);


void usart3_config(void);
uint8_t UART3_Put_Char(unsigned char DataToSend);


//通过串口2向上位机发送 数据包
void usart2_config(void);
void Uart2_Put_Buf(u8 data_to_send[],u8 _cnt);


#endif
