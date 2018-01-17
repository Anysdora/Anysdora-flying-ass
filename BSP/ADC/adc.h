#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"

// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue;


void ADC1_Init(void);


#endif /* __ADC_H */

