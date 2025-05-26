/**
 * @file sensors.h
 * @brief Thu vien doc cam bien
 */

#ifndef __SENSORS_H
#define __SENSORS_H

#include "stm32f10x.h"
#include "adc.h"
#include "filter.h"
#include <math.h>
#include <stddef.h>

/* Dinh nghia hang so chuyen doi */
#define SOLAR_VOLTAGE_RATIO     21.1f  /* He so chia ap tam pin (R1+R2)/R2 */
#define BATTERY_VOLTAGE_RATIO   34.5f  /* He so chia ap pin */
#define LOAD_VOLTAGE_RATIO      31.8f  /* He so chia ap tai */
#define DC_VOLTAGE_RATIO        31.3f  /* He so chia ap DC */

/* Hang so chuyen doi nhiet do NTC */
#define NTC_RESISTANCE          10000.0f
#define NTC_B_COEFFICIENT       3950.0f
#define NTC_T0                  298.15f  /* 25°C in Kelvin */

/* Hang so chuyen doi dong dien */
#define CURRENT_SENSOR_SENS     0.066f  /* mV/mA (tuy thuoc vao cam bien) */
#define CURRENT_SENSOR_OFFSET   2.5f    /* Dien ap offset (V) */

/* Cau truc chua gia tri cac cam bien */
typedef struct {
    float solarVoltage;     /* Dien ap tam pin (V) */
    float solarCurrent;     /* Dong tam pin (A) */
    float batteryVoltage;   /* Dien ap pin (V) */
    float batteryCurrent;   /* Dong sac pin (A) */
    float loadVoltage;      /* Dien ap tai (V) */
    float loadCurrent;      /* Dong tai (A) */
    float temperature;      /* Nhiet do (°C) */
    uint32_t power;         /* Cong suat (W) */
} SensorValues_t;

/* Ham khoi tao */
void Sensors_Init(void);

/* Ham doc gia tri cam bien */
void Sensors_ReadAll(SensorValues_t *values);

/* Ham doc gia tri rieng le */
float Sensors_GetSolarVoltage(void);
float Sensors_GetSolarCurrent(void);
float Sensors_GetBatteryVoltage(void);
float Sensors_GetBatteryCurrent(void);
float Sensors_GetLoadVoltage(void);
float Sensors_GetLoadCurrent(void);
float Sensors_GetTemperature(void);
uint32_t Sensors_GetPower(void);

#endif /* __SENSORS_H */