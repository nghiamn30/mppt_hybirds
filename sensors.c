/**
 * @file sensors.c
 * @brief Thuc thi thu vien doc cam bien
 */

#include "sensors.h"

/* Khai bao cac bien static */
static SensorValues_t sensorValues = {0};
static KalmanFilter voltageFilters[4]; /* Bo loc cho Solar, Battery, Load, DC */
static KalmanFilter currentFilters[2]; /* Bo loc cho Solar current, Load current */
static KalmanFilter temperatureFilter; /* Bo loc cho Temperature */

/**
 * @brief Khoi tao cac bo loc cho cam bien
 */
void Sensors_Init(void) {
    uint8_t i;
    
    /* Khoi tao bo loc Kalman cho cac cam bien */
    for(i = 0; i < 4; i++) {
        kalman_init(&voltageFilters[i], 0.01f, 0.1f); /* Q=0.01 (process), R=0.1 (measurement) */
    }
    
    for(i = 0; i < 2; i++) {
        kalman_init(&currentFilters[i], 0.05f, 0.2f); /* Q=0.05, R=0.2 - dong dien thay doi nhanh hon */
    }
    
    kalman_init(&temperatureFilter, 0.001f, 0.1f); /* Q=0.001, R=0.1 - nhiet do thay doi cham */
    
    /* Khoi tao ADC */
    ADC_Initialize();
}

/**
 * @brief Doc tat ca gia tri cam bien
 * @param values Con tro den cau truc chua gia tri cam bien
 */
void Sensors_ReadAll(SensorValues_t *values) {
    /* Khai bao bien */
    float raw_solar_voltage;
    float raw_solar_current;
    float raw_battery_voltage;
    float raw_battery_current;
    float raw_load_voltage;
    float raw_load_current;
    float raw_temperature;
    float filtered_solar_voltage;
    float filtered_solar_current;
    float filtered_battery_voltage;
    float filtered_battery_current;
    float filtered_load_voltage;
    float filtered_load_current;
    float filtered_temperature;
    float resistance;
    
    /* Doc gia tri ADC */
    raw_solar_voltage = ADC_GetVoltage(0);
    raw_solar_current = ADC_GetVoltage(1);
    raw_battery_voltage = ADC_GetVoltage(2);
    raw_battery_current = ADC_GetVoltage(3);
    raw_load_voltage = ADC_GetVoltage(4);
    raw_load_current = ADC_GetVoltage(5);
    raw_temperature = ADC_GetVoltage(6);
    
    /* Loc gia tri ADC */
    filtered_solar_voltage = kalman_update(&voltageFilters[0], raw_solar_voltage);
    filtered_solar_current = kalman_update(&currentFilters[0], raw_solar_current);
    filtered_battery_voltage = kalman_update(&voltageFilters[1], raw_battery_voltage);
    filtered_battery_current = kalman_update(&currentFilters[1], raw_battery_current);
    filtered_load_voltage = kalman_update(&voltageFilters[2], raw_load_voltage);
    filtered_load_current = kalman_update(&currentFilters[1], raw_load_current);
    filtered_temperature = kalman_update(&temperatureFilter, raw_temperature);
    
    /* Chuyen doi gia tri ADC sang gia tri vat ly */
    values->solarVoltage = filtered_solar_voltage * SOLAR_VOLTAGE_RATIO;
    values->batteryVoltage = filtered_battery_voltage * BATTERY_VOLTAGE_RATIO;
    values->loadVoltage = filtered_load_voltage * LOAD_VOLTAGE_RATIO;
    
    /* Chuyen doi dong dien - gia dinh dung cam bien ACS712 */
    values->solarCurrent = (filtered_solar_current - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENS;
    values->batteryCurrent = (filtered_battery_current - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENS;
    values->loadCurrent = (filtered_load_current - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENS;
    
    /* Chuyen doi nhiet do - su dung NTC */
    resistance = NTC_RESISTANCE * (3.3f / filtered_temperature - 1.0f);
    values->temperature = 1.0f / ((float)log(resistance / NTC_RESISTANCE) / NTC_B_COEFFICIENT + 1.0f / NTC_T0) - 273.15f;
    
    /* Tinh cong suat */
    values->power = (uint32_t)(values->solarVoltage * values->solarCurrent);
    
    /* Luu gia tri vao bien static de su dung cho cac ham doc gia tri rieng le */
    sensorValues = *values;
}

/**
 * @brief Doc dien ap tam pin
 * @return Dien ap tam pin (V)
 */
float Sensors_GetSolarVoltage(void) {
    return sensorValues.solarVoltage;
}

/**
 * @brief Doc dong dien tam pin
 * @return Dong dien tam pin (A)
 */
float Sensors_GetSolarCurrent(void) {
    return sensorValues.solarCurrent;
}

/**
 * @brief Doc dien ap pin
 * @return Dien ap pin (V)
 */
float Sensors_GetBatteryVoltage(void) {
    return sensorValues.batteryVoltage;
}

/**
 * @brief Doc dong sac pin
 * @return Dong sac pin (A)
 */
float Sensors_GetBatteryCurrent(void) {
    return sensorValues.batteryCurrent;
}

/**
 * @brief Doc dien ap tai
 * @return Dien ap tai (V)
 */
float Sensors_GetLoadVoltage(void) {
    return sensorValues.loadVoltage;
}

/**
 * @brief Doc dong tai
 * @return Dong tai (A)
 */
float Sensors_GetLoadCurrent(void) {
    return sensorValues.loadCurrent;
}

/**
 * @brief Doc nhiet do
 * @return Nhiet do (°C)
 */
float Sensors_GetTemperature(void) {
    return sensorValues.temperature;
}

/**
 * @brief Tinh cong suat
 * @return Cong suat (W)
 */
uint32_t Sensors_GetPower(void) {
    return sensorValues.power;
}