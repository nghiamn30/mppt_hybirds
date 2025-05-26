#include "adc.h"

/* Bien de luu gia tri ADC */
uint16_t u16ValueAdc1Channel[NUMBER_OF_ADC_CHANNEL] = {0U}; /* PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 */
KalmanFilter adc_filters[NUMBER_OF_ADC_CHANNEL]; /* Bo loc Kalman cho moi kenh ADC */

/* Ham khoi tao tong the */
void RD_ADC_Init(void)
{
    /* Khoi tao cac bo loc Kalman cho ADC */
    RD_ADC_InitFilters();
    
    /* Cau hinh DMA va ADC */
    RD_DMA_ConfigChannel_11((uint32_t *)ADC1_DR_ADDRESS, (uint32_t *)u16ValueAdc1Channel, NUMBER_OF_ADC_CHANNEL);
    RD_ADC_Config();
}

/* Ham khoi tao cac bo loc Kalman cho ADC */
void RD_ADC_InitFilters(void)
{
    uint8_t i;
    for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
    {
        /* Khoi tao bo loc voi process_noise = 10.0, measurement_noise = 5.0 */
        /* Tham so nay co the dieu chinh de phu hop voi ung dung */
        kalman_init(&adc_filters[i], 10.0, 5.0);
    }
}

/* Ham cau hinh ADC */
void RD_ADC_Config(void)
{
    /* Bat clock cho ADC1 va GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Cau hinh cac chan analog: PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;  /* Bat che do quet vi co 8 kenh */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 8;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* Cau hinh 8 kenh ADC */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_41Cycles5); /* PA0 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_41Cycles5); /* PA1 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_41Cycles5); /* PA2 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_41Cycles5); /* PA3 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5); /* PA4 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5); /* PA5 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_41Cycles5); /* PA6 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5); /* PA7 */
    
    /* Bat ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* Bat DMA cho ADC */
    ADC_DMACmd(ADC1, ENABLE);
    
    /* Hieu chuan ADC1 */
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    /* Bat dau chuyen doi ADC1 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/* Ham cau hinh DMA cho ADC */
void RD_DMA_ConfigChannel_11(uint32_t *pStartAddress, uint32_t *pDestination, uint32_t u32NumberDataTranfer)
{
    /* Bat clock cho DMA1 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    /* Thiet lap dia chi thanh ghi ngoai vi */
    DMA1_Channel1->CPAR = (uint32_t)pStartAddress;
    
    /* Thiet lap dia chi bo nho dich */
    DMA1_Channel1->CMAR = (uint32_t)pDestination;
    
    /* Cau hinh so luong du lieu can chuyen */
    DMA1_Channel1->CNDTR = u32NumberDataTranfer;
    
    /* Cau hinh thanh ghi dieu khien DMA */
    DMA1_Channel1->CCR |= 0x25A0;
    
    /* Kich hoat kenh DMA */
    DMA1_Channel1->CCR |= 0x01;
}

/* Ham doc gia tri ADC da loc */
uint16_t RD_ADC_GetFilteredValue(uint8_t channel)
{
    /* Loc gia tri ADC tho bang Kalman filter */
    float filtered_value = kalman_update(&adc_filters[channel], (float)u16ValueAdc1Channel[channel]);
    
    /* Chuyen ve kieu uint16_t, dam bao trong khoang 0-4095 */
    if(filtered_value < 0) filtered_value = 0;
    if(filtered_value > 4095) filtered_value = 4095;
    
    return (uint16_t)filtered_value;
}

/* Ham doc nhiet do tu ADC */
float RD_ADC_GetTemperature(void)
{
    /* Doc gia tri ADC da loc tu kenh 7 (PA7) */
    uint16_t adcValue = RD_ADC_GetFilteredValue(7);
    
    float _TSlog = 0;
    float _TemperatureVal = 0;
    
    /* Chuyen doi gia tri ADC thanh nhiet do su dung cong thuc Steinhart-Hart */
    _TSlog = log(NTC_RESISTANCE * (4095.00 / (float)adcValue - 1.00));
    _TemperatureVal = (1.0 / (1.009249522e-03 + 2.378405444e-04 * _TSlog + 2.019202697e-07 * _TSlog * _TSlog * _TSlog)) - 273.15;
    
    /* Kiem tra gia tri hop le */
    if (_TemperatureVal < 0)
    {
        _TemperatureVal = 11;
    }
    
    return _TemperatureVal;
}

/* Ham chuyen doi ADC sang dien ap (V) */
float RD_ADC_ConvertToVoltage(uint16_t adcValue)
{
    return (adcValue * ADC_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
}

/* Ham doc dien ap voi he so phan ap tuy chinh */
float RD_ADC_GetScaledVoltage(uint16_t adcValue, float dividerRatio)
{
    float voltage = RD_ADC_ConvertToVoltage(adcValue);
    return voltage * dividerRatio;
}

/* Ham doc dien ap pin mat troi (PA1) */
float RD_ADC_GetSolarVoltage(void)
{
    uint16_t filtered_adc = RD_ADC_GetFilteredValue(1);
    return RD_ADC_GetScaledVoltage(filtered_adc, VOLTAGE_DIVIDER_RATIO);
}

/* Ham doc dien ap pin (PA2) */
float RD_ADC_GetBatteryVoltage(void)
{
    uint16_t filtered_adc = RD_ADC_GetFilteredValue(2);
    return RD_ADC_GetScaledVoltage(filtered_adc, 34.5);
}

/* Ham doc dien ap tai (PA3) */
float RD_ADC_GetLoadVoltage(void)
{
    uint16_t filtered_adc = RD_ADC_GetFilteredValue(3);
    return RD_ADC_GetScaledVoltage(filtered_adc, 31.5);
}

/* Ham doc dien ap DC ngoai (PA5) */
float RD_ADC_GetDCVoltage(void)
{
    uint16_t filtered_adc = RD_ADC_GetFilteredValue(5);
    return RD_ADC_GetScaledVoltage(filtered_adc, 31.7);
}

/* Ham doc dong dien tai (PA4)*/
float RD_ADC_GetLoadCurrent(void)
{
    /* Doc gia tri dien ap V3 da loc tu chan PA6 */
    uint16_t filtered_adc = RD_ADC_GetFilteredValue(4);
    float Vout = RD_ADC_ConvertToVoltage(filtered_adc)/1.1;
    
    /* Ap dung cong thuc: Vout = 9.9099 * V3 + 0.3270207 */
    float V3 = (Vout - ILOAD_OFFSET) / ILOAD_GAIN;
    
    /* Chuyen doi Vout thanh dong dien */
    
    /* Thay doi he so nay phu thuoc vao cam bien thuc te */
    float current = V3 / 0.05f; /* Don vi: Ampe */
    
    return current;
}
