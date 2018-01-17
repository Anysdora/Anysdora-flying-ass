#ifndef __LED_H
#define	__LED_H
#include "stm32f10x.h"




#define magnet_ON GPIO_SetBits(GPIOD,GPIO_Pin_13);
#define magnet_OFF GPIO_ResetBits(GPIOD,GPIO_Pin_13);




/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1

#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_10);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_10)
					
#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_11);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_11)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_10);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_10)

#define LED4(a)	if (a)	\
					GPIO_SetBits(GPIOD,GPIO_Pin_11);\
					else		\
					GPIO_ResetBits(GPIOD,GPIO_Pin_11)				
										
void LED_Init(void);
void LED_Sailing(void);

//ledœ‘ æ¥¶¿Ì
void led_handle(void);

#endif /* __LED_H */
