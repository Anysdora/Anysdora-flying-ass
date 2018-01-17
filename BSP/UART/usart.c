#include "usart.h"
#include <stdarg.h>

void usart1_config(void)  //PA9 PA10
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;                 /*设置波特率为115200*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
    USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
    /*完成串口COM1的时钟配置、GPIO配置，根据上述参数初始化并使能*/
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);			 //接收中断
	
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
uint8_t UART1_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART1->SR & USART_FLAG_TXE));
	return DataToSend;
}


/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART1_printf()调用
 */
static char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

/*
 * 函数名：USART_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里只用到了串口1，即USART1
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应用USART_printf( USART1, "\r\n this is a demo \r\n" );
 *            		 USART_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART_printf( USART1, "\r\n %s \r\n", j );
 */
void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // 判断是否到达字符串结束符
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //字符串
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//十进制
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}


void usart3_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO功能的时钟
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3 ,ENABLE);  //进行重映射

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART3 Rx (PD.09) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;                 /*设置波特率为9600*/
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
	USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
	/*完成串口COM3的时钟配置、GPIO配置，根据上述参数初始化并使能*/

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		void UART2_Put_Char(unsigned char DataToSend)
*功　　能:		蓝牙发送一个字节
*输入参数：
		unsigned char DataToSend   要发送的字节数据
*输出参数：没有	
*******************************************************************************/
uint8_t UART2_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART2, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART2->SR & USART_FLAG_TXE));
	return DataToSend;
}

//通过串口2向上位机发送 数据包
void Uart2_Put_Buf(u8 data_to_send[],u8 _cnt)
{
	u8 i;
	for(i=0;i<_cnt;i++)
	{
		USART2->DR = data_to_send[i];
	  	while (!(USART2->SR & USART_FLAG_TXE));	 		//等待发送完成
	}	
}


void usart2_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART2 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO功能的时钟
	GPIO_PinRemapConfig(GPIO_Remap_USART2 ,ENABLE);  //进行重映射

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure USART2 Rx (PD.06) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;                 /*设置波特率为115200*/
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
	USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
	/*完成串口COM2的时钟配置、GPIO配置，根据上述参数初始化并使能*/

	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART2, ENABLE);
}
