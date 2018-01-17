#ifndef __MS5611_H
#define	__MS5611_H
#include "stm32f10x.h"

/* MS5611 Register Address ------------------------------------------------------------*/
#define MS561101BA_ADC_RD          0x00
#define	MS561101BA_PROM_RD 	       0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611µÄµØÖ·
#define MS561101BA_RST             0x1E  //cmd ¸´Î»

#define	MS561101BA_D2_OSR_4096     0x58	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS561101BA_D1_OSR_4096     0x48

#define MS5611_OSR256			   0x40
#define MS5611_OSR512	 		   0x42
#define MS5611_OSR1024			   0x44
#define MS5611_OSR2048			   0x46
#define MS5611_OSR4096			   0x48

#define FILTER_num 20

u8  MS5611_init(void);
float Get_High(void);

extern uint32_t D1_Pres;

#endif
