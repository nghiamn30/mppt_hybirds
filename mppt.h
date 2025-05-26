/**
 * @file mppt.h
 * @brief Thu vien dieu khien MPPT
 */

#ifndef __MPPT_H
#define __MPPT_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>

/* Dinh nghia trang thai MPPT */
typedef enum {
    MPPT_STATE_IDLE,           /* Trang thai cho */
    MPPT_STATE_START,          /* Bat dau MPPT */
    MPPT_STATE_TRACKING,       /* Dang tim diem cong suat toi da */
    MPPT_STATE_CONSTANT_VOLT,  /* Duy tri dien ap hang */
    MPPT_STATE_FLOAT,          /* Duy tri dien ap float */
    MPPT_STATE_ERROR           /* Loi */
} MPPT_State_t;

typedef enum {
    MPPT_STATUS_OK,
    MPPT_STATUS_ERROR,
    MPPT_STATUS_TIMEOUT
} MPPT_Status_t;

/* Cau truc chua thong so MPPT */
typedef struct {
    float solarVoltage;      /* Dien ap tam pin (V) */
    float solarCurrent;      /* Dong dien tam pin (A) */
    float batteryVoltage;    /* Dien ap pin (V) */
    float dutyCycle;         /* Duty cycle (0.0-1.0) */
    float maxPower;          /* Cong suat toi da (W) */
    uint32_t power;          /* Cong suat hien tai (W) */
    MPPT_State_t state;      /* Trang thai MPPT */
} MPPT_Params_t;

/* Cau truc chua nguong MPPT */
typedef struct {
    float batteryVoltageMin;   /* Dien ap pin toi thieu (V) */
    float batteryVoltageMax;   /* Dien ap pin toi da (V) */
    float batteryVoltageFloat; /* Dien ap pin float (V) */
    float solarVoltageMin;     /* Dien ap tam pin toi thieu (V) */
    float minDutyCycle;        /* Duty cycle toi thieu */
    float maxDutyCycle;        /* Duty cycle toi da */
    uint16_t mpptInterval;     /* Thoi gian cap nhat MPPT (ms) */
} MPPT_Thresholds_t;

/* Ham khoi tao */
void MPPT_Init(uint32_t pwmFrequency, uint16_t deadTime);

/* Ham dat thong so */
void MPPT_SetThresholds(MPPT_Thresholds_t *thresholds);
void MPPT_SetInputs(float solarVoltage, float solarCurrent, float batteryVoltage);

/* Ham dieu khien */
void MPPT_Start(void);
void MPPT_Stop(void);
void MPPT_Process(void);

/* Ham lay trang thai */
MPPT_State_t MPPT_GetState(void);
MPPT_Params_t MPPT_GetParams(void);
float MPPT_GetDutyCycle(void);

#endif /* __MPPT_H */
