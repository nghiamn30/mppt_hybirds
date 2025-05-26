/**
 * @file mppt.c
 * @brief Thuc thi thu vien MPPT v?i ch? d? can gi?a PWM
 */

#include "mppt.h"
#include "utils.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

/* Bien toan cuc */
static MPPT_Params_t mpptParams = {0};
static MPPT_Thresholds_t mpptThresholds = {0};

/* Bien cho thuat toan MPPT */
static float lastPower = 0;
static float lastDutyCycle = 0.5f;
static int8_t mpptDirection = 1; /* 1: tang, -1: giam */
static uint32_t lastMpptTime = 0;

/**
 * @brief Khoi tao bo dieu khien MPPT v?i ch? d? can gi?a PWM
 * @param pwmFrequency Tan so PWM (kHz)
 * @param deadTime Thoi gian chet (ns)
 */
void MPPT_Init(uint32_t pwmFrequency, uint16_t deadTime) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    uint16_t timerPeriod;
    
    /* Khoi tao cac gia tri */
    mpptParams.state = MPPT_STATE_IDLE;
    mpptParams.dutyCycle = 0.4f;
    mpptParams.maxPower = 0;
    
    /* Thiet lap cac nguong mac dinh */
    mpptThresholds.batteryVoltageMin = 11.0f;  /* 11V */
    mpptThresholds.batteryVoltageMax = 14.4f;  /* 14.4V */
    mpptThresholds.batteryVoltageFloat = 13.5f; /* 13.5V */
    mpptThresholds.solarVoltageMin = 12.0f;    /* 12V */
    mpptThresholds.minDutyCycle = 0.1f;        /* 10% */
    mpptThresholds.maxDutyCycle = 0.8f;        /* 80% */
    mpptThresholds.mpptInterval = 1000;        /* 1s */
    
    /* Bat clock cho TIM1 va GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | 
                          RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    /* Cau hinh GPIO cho PWM */
    /* PA10 - TIM1_CH3 (high-side) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* PB15 - TIM1_CH3N (low-side) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Cau hinh TIM1 - CH? Ð? CAN GI?A */
    /* Tinh toan gia tri dem cho tan so PWM, di?u ch?nh cho ch? d? can gi?a */
    timerPeriod = (uint16_t)((SystemCoreClock / (2 * pwmFrequency * 1000)) - 1);
    
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_Period = timerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    /* Cau hinh PWM */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (uint16_t)(mpptParams.dutyCycle * timerPeriod);
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    
    /* Cau hinh deadtime */
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = (uint8_t)((SystemCoreClock / 1000000) * deadTime / 1000);
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    
    /* Bat preload */
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    
    /* Bat Timer */
    TIM_Cmd(TIM1, ENABLE);
    
    /* Tat dau ra PWM */
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

/**
 * @brief Dat cac nguong cho MPPT
 * @param thresholds Con tro den cau truc chua nguong
 */
void MPPT_SetThresholds(MPPT_Thresholds_t *thresholds) {
    if (thresholds != 0) {
        mpptThresholds = *thresholds;
    }
}

/**
 * @brief Cap nhat cac dau vao cho MPPT
 * @param solarVoltage Dien ap tam pin (V)
 * @param solarCurrent Dong dien tam pin (A)
 * @param batteryVoltage Dien ap pin (V)
 */
void MPPT_SetInputs(float solarVoltage, float solarCurrent, float batteryVoltage) {
    mpptParams.solarVoltage = solarVoltage;
    mpptParams.solarCurrent = solarCurrent;
    mpptParams.batteryVoltage = batteryVoltage;
    
    /* Tinh cong suat */
    mpptParams.power = (uint32_t)(solarVoltage * solarCurrent);
    
    /* Cap nhat cong suat toi da neu can */
    if ((float)mpptParams.power > mpptParams.maxPower) {
        mpptParams.maxPower = (float)mpptParams.power;
    }
}

/**
 * @brief Bat chuc nang MPPT
 */
void MPPT_Start(void) {
    mpptParams.state = MPPT_STATE_START;
    lastPower = 0;
    lastDutyCycle = mpptParams.dutyCycle;
    mpptDirection = 1;
    lastMpptTime = GetMillis();
    
    /* Tinh duty cycle ban dau dua tren ty le dien ap */
    mpptParams.dutyCycle = mpptParams.batteryVoltage / mpptParams.solarVoltage;
    
    /* Gioi han duty cycle */
    if (mpptParams.dutyCycle < mpptThresholds.minDutyCycle) mpptParams.dutyCycle = mpptThresholds.minDutyCycle;
    if (mpptParams.dutyCycle > mpptThresholds.maxDutyCycle) mpptParams.dutyCycle = mpptThresholds.maxDutyCycle;
    
    /* Cap nhat gia tri PWM */
    TIM_SetCompare3(TIM1, (uint16_t)(mpptParams.dutyCycle * TIM1->ARR));
    
    /* Bat dau ra PWM */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
 * @brief Tat chuc nang MPPT
 */
void MPPT_Stop(void) {
    mpptParams.state = MPPT_STATE_IDLE;
    
    /* Tat dau ra PWM */
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

/**
 * @brief Xu ly thuat toan MPPT
 */
void MPPT_Process(void) {
    float powerChange;
    float dutyChange;
    
    /* Kiem tra dieu kien an toan */
    /*if (mpptParams.batteryVoltage < mpptThresholds.batteryVoltageMin ||
        mpptParams.solarVoltage < mpptThresholds.solarVoltageMin) {
        MPPT_Stop();
        return;
    }*/
    
    switch (mpptParams.state) {
        case MPPT_STATE_IDLE:
            /* Khong lam gi */
            break;
            
        case MPPT_STATE_START:
            /* Chuyen sang trang thai theo doi */
            mpptParams.state = MPPT_STATE_TRACKING;
            break;
            
        case MPPT_STATE_TRACKING:
            /* Kiem tra dieu kien chuyen sang dien ap hang */
            if (mpptParams.batteryVoltage >= mpptThresholds.batteryVoltageMax) {
                mpptParams.state = MPPT_STATE_CONSTANT_VOLT;
                break;
            } 
            
            /* Thuc hien cap nhat MPPT theo khoang thoi gian */
            if (GetMillis() - lastMpptTime >= mpptThresholds.mpptInterval) {
                /* Thuc hien thuat toan Perturb & Observe */
                if (lastPower > 0) {
                    powerChange = (float)mpptParams.power - lastPower;
                    dutyChange = mpptParams.dutyCycle - lastDutyCycle;
                    
                    if (powerChange > 0) {
                        /* Cong suat tang */
                        if (dutyChange > 0) {
                            /* Tiep tuc tang duty */
                            mpptDirection = 1;
                        } else {
                            /* Tiep tuc giam duty */
                            mpptDirection = -1;
                        }
                    } else if (powerChange < 0) {
                        /* Cong suat giam */
                        if (dutyChange > 0) {
                            /* Doi huong - giam duty */
                            mpptDirection = -1;
                        } else {
                            /* Doi huong - tang duty */
                            mpptDirection = 1;
                        }
                    }
                    /* Neu powerChange = 0, giu nguyen huong */
                }
                
                /* Luu tru cac gia tri hien tai */
                lastPower = (float)mpptParams.power;
                lastDutyCycle = mpptParams.dutyCycle;
                
                /* Dieu chinh duty cycle */
                mpptParams.dutyCycle += mpptDirection * 0.01f;
                
                /* Gioi han duty cycle */
                if (mpptParams.dutyCycle < mpptThresholds.minDutyCycle) mpptParams.dutyCycle = mpptThresholds.minDutyCycle;
                if (mpptParams.dutyCycle > mpptThresholds.maxDutyCycle) mpptParams.dutyCycle = mpptThresholds.maxDutyCycle;
                
                /* Cap nhat gia tri PWM */
                TIM_SetCompare3(TIM1, (uint16_t)(mpptParams.dutyCycle * TIM1->ARR));
                
                /* Cap nhat thoi gian */
                lastMpptTime = GetMillis();
            }
            break;
            
        case MPPT_STATE_CONSTANT_VOLT:
            /* Duy tri dien ap pin o muc toi da */
            if (mpptParams.batteryVoltage > mpptThresholds.batteryVoltageMax) {
                /* Giam duty cycle */
                mpptParams.dutyCycle -= 0.01f;
                if (mpptParams.dutyCycle < mpptThresholds.minDutyCycle) mpptParams.dutyCycle = mpptThresholds.minDutyCycle;
                
                /* Cap nhat gia tri PWM */
                TIM_SetCompare3(TIM1, (uint16_t)(mpptParams.dutyCycle * TIM1->ARR));
            } else if (mpptParams.batteryVoltage < mpptThresholds.batteryVoltageMax - 0.2f) {
                /* Dien ap giam qua nhieu, tang duty cycle */
                mpptParams.dutyCycle += 0.01f;
                if (mpptParams.dutyCycle > mpptThresholds.maxDutyCycle) mpptParams.dutyCycle = mpptThresholds.maxDutyCycle;
                
                /* Cap nhat gia tri PWM */
                TIM_SetCompare3(TIM1, (uint16_t)(mpptParams.dutyCycle * TIM1->ARR));
                
                /* Neu dien ap giam qua thap, chuyen ve trang thai theo doi */
                if (mpptParams.batteryVoltage < mpptThresholds.batteryVoltageMax - 0.5f) {
                    mpptParams.state = MPPT_STATE_TRACKING;
                }
            }
            break;
            
        case MPPT_STATE_FLOAT:
            /* Do thuc te, ham nay chua duoc su dung trong MPPT don gian nay */
            break;
            
        case MPPT_STATE_ERROR:
            /* Do thuc te, ham nay chua duoc su dung trong MPPT don gian nay */
            break;
            
        default:
            /* Trang thai khong xac dinh, ve trang thai cho */
            mpptParams.state = MPPT_STATE_IDLE;
            break;
    }
}

/**
 * @brief Lay trang thai hien tai cua MPPT
 * @return Trang thai hien tai
 */
MPPT_State_t MPPT_GetState(void) {
    return mpptParams.state;
}

/**
 * @brief Lay thong so hien tai cua MPPT
 * @return Cau truc chua thong so
 */
MPPT_Params_t MPPT_GetParams(void) {
    return mpptParams;
}

/**
 * @brief Lay duty cycle hien tai
 * @return Duty cycle (0.0-1.0)
 */
float MPPT_GetDutyCycle(void) {
    return mpptParams.dutyCycle;
}
