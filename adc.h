/**
 * @file adc.h
 * @brief Thu vien xu ly ADC
 */

#ifndef _ADC_H
#define _ADC_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "filter.h"

/* Dinh nghia hang so */
#define ADC1_DR_ADDRESS          ((uint32_t)0x4001244C)
#define NUMBER_OF_ADC_CHANNEL    8U  /* Giu nguyen 8 kenh */
#define NTC_RESISTANCE           10000.0
#define ADC_REFERENCE_VOLTAGE    3.3f  /* Dien ap tham chieu (V) */
#define ADC_RESOLUTION           4096  /* 2^12 levels (12-bit ADC) */
#define VOLTAGE_DIVIDER_RATIO    21.1f /* He so phan ap voi R1=20k, R2=1k */

/* Hang so cho cong thuc tinh dong dien Iload moi */
#define ILOAD_GAIN              9.9099f   /* He so khuech dai */
#define ILOAD_OFFSET            0.3270207f /* Dien ap offset */

/* Khai bao bien global */
extern uint16_t u16ValueAdc1Channel[NUMBER_OF_ADC_CHANNEL]; /* PA0 -> PA7 */
extern KalmanFilter adc_filters[NUMBER_OF_ADC_CHANNEL]; /* Bo loc cho moi kenh ADC */

/* Cac ham cau hinh va khoi tao */
void RD_ADC_Init(void);
void RD_ADC_Config(void);
void RD_DMA_ConfigChannel_11(uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTranfer);
void RD_ADC_InitFilters(void);

/* Ham doc gia tri ADC da loc */
uint16_t RD_ADC_GetFilteredValue(uint8_t channel);

/* Cac ham doc va xu ly gia tri ADC */
float RD_ADC_GetTemperature(void);
float RD_ADC_ConvertToVoltage(uint16_t adcValue);
float RD_ADC_GetScaledVoltage   (uint16_t adcValue, float dividerRatio);

/* Cac ham doc gia tri cu the */
float RD_ADC_GetSolarVoltage(void);      /* PA1 - kenh 1 */
float RD_ADC_GetBatteryVoltage(void);    /* PA2 - kenh 2 */
float RD_ADC_GetLoadVoltage(void);       /* PA3 - kenh 3 */
float RD_ADC_GetDCVoltage(void);         /* PA5 - kenh 5 */
float RD_ADC_GetLoadCurrent(void);       /* PA6 - kenh 6 (ham moi) */

#endif /* __RD_ADC_H */
