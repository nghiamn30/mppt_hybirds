#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "filter.h"
#include "adc.h"
#include "mppt.h"
#include "utils.h"
#include "control_mosfet.h"

/* Bien trang thai he thong */
typedef enum {
    SYSTEM_STATE_INIT,       /* Trang thai khoi tao */
    SYSTEM_STATE_RUNNING,    /* Trang thai dang chay */
    SYSTEM_STATE_ERROR,      /* Trang thai loi */
    SYSTEM_STATE_LOW_POWER   /* Trang thai tiet kiem dien */
} SystemState_t;

/* Function prototypes (Khai bao ham) */
void CheckSystemConditions(void);

/* Khai bao bien */
static SystemState_t systemState = SYSTEM_STATE_INIT;
static uint32_t lastSystemCheckTime = 0;
static uint8_t errorCount = 0;

/* Timer variables cho non-blocking delays */
static uint32_t errorStateStartTime = 0;
static uint32_t lowPowerStateStartTime = 0;
static uint8_t errorStateFirstEntry = 1;
static uint8_t lowPowerStateFirstEntry = 1;

/* Gia tri doc tu ADC */
static float temperature = 0;
static float batteryVoltage = 0;
static float solarVoltage = 0;
static float loadVoltage = 0;
static float dcVoltage = 0;
static float loadCurrent = 0;
static int dem = 0;
/* System timing variables */
static uint32_t systemStartTime = 0;
static uint32_t systemRunningTime = 0;

/* Ham kiem tra dieu kien he thong */
void CheckSystemConditions(void)
{
    /* Doc cac gia tri da duoc loc o muc ADC */
    temperature = RD_ADC_GetTemperature();
    solarVoltage = RD_ADC_GetSolarVoltage();
    batteryVoltage = RD_ADC_GetBatteryVoltage();
    loadVoltage = RD_ADC_GetLoadVoltage();
    dcVoltage = RD_ADC_GetDCVoltage();
    loadCurrent = RD_ADC_GetLoadCurrent ();
    
    /* Kiem tra dieu kien loi */
    if (temperature > 75.0f) { /* Qua nhiet */
        systemState = SYSTEM_STATE_ERROR;
        errorCount++;
    }   
/*		else if (batteryVoltage > 0.01f) {  Dien ap pin qua thap 
        systemState = SYSTEM_STATE_LOW_POWER;
    } */
    else if (solarVoltage < 5.0f) { /* Khong co dien ap tu tam pin */
        systemState = SYSTEM_STATE_LOW_POWER;
    }
    else {
        if (systemState != SYSTEM_STATE_RUNNING) {
            systemState = SYSTEM_STATE_RUNNING;
            /* Reset cac timer khi chuyen ve RUNNING */
            errorStateFirstEntry = 1;
            lowPowerStateFirstEntry = 1;
            /* Khoi dong lai MPPT neu truoc do khong chay */
            MPPT_Start();
        }
    }
}

int main(void)
{
    /* Khoi tao cac module */
    TimerInit(); /* Khoi tao SysTick tu utils.c */
    RD_ADC_Init(); /* Khoi tao ADC */
    
    /* Khoi tao MPPT voi tan so 20kHz va deadtime 1000ns */
    MPPT_Init(24, 5000);
    
		/*Khoi tao cac chan dieu khien mosfet */
		InitControlMosfet();
		setMosfet(0,0,0,0,0);
    /* Thiet lap nguong cho MPPT */
	
    MPPT_Thresholds_t thresholds;
    thresholds.batteryVoltageMin = 11.0f;
    thresholds.batteryVoltageMax = 14.4f;
    thresholds.batteryVoltageFloat = 13.5f;
    thresholds.solarVoltageMin = 12.0f;
    thresholds.minDutyCycle = 0.1f;
    thresholds.maxDutyCycle = 0.8f;
    thresholds.mpptInterval = 1000;
    MPPT_SetThresholds(&thresholds);
    
    /* Khoi dong he thong */
    systemState = SYSTEM_STATE_INIT;
    systemStartTime = GetMillis();
    lastSystemCheckTime = systemStartTime;
    
    /* Vong lap chinh */
    while(1)
    {
        
        /* Cap nhat system running time */
        systemRunningTime = GetMillis() - systemStartTime;
        
        /* Kiem tra dieu kien he thong moi 2000ms (2 giay) */
        if (GetMillis() - lastSystemCheckTime >= 3000) {
            CheckSystemConditions();
            lastSystemCheckTime = GetMillis();
        }
        
        /* Xu ly theo trang thai - NON-BLOCKING STATE MACHINE */
        switch (systemState) {
            case SYSTEM_STATE_INIT:
                /* Khoi tao he thong da hoan tat, chuyen sang chay */
                systemState = SYSTEM_STATE_RUNNING;
                break;
                
            case SYSTEM_STATE_RUNNING:
                /* Cap nhat thong so vao MPPT */
                MPPT_SetInputs(solarVoltage, loadCurrent, batteryVoltage);
                
                /* Xu ly MPPT */
                MPPT_Process();
                break;
                
            case SYSTEM_STATE_ERROR:
                /* Lan dau vao error state */
                if (errorStateFirstEntry) {
                    MPPT_Stop(); /* Dung MPPT trong truong hop loi */
                    errorStateStartTime = GetMillis();
                    errorStateFirstEntry = 0;
                }
                
                /* Cho 5 giay khong blocking */
                if (GetMillis() - errorStateStartTime >= 5000) {
                    /* Neu da co qua nhieu loi, reset he thong */
                    if (errorCount > 5) {
                        /* Reset he thong hoac chuyen sang che do an toan */
                        NVIC_SystemReset();
                    }
                    
                    /* Thu khoi dong lai */
                    systemState = SYSTEM_STATE_INIT;
                    errorStateFirstEntry = 1; /* Reset flag */
                }
                break;
                
            case SYSTEM_STATE_LOW_POWER:
                /* Lan dau vao low power state */
                if (lowPowerStateFirstEntry) {
                    MPPT_Stop(); /* Dung MPPT de tiet kiem nang luong */
                    lowPowerStateStartTime = GetMillis();
                    lowPowerStateFirstEntry = 0;
                    
                    /* Cau hinh che do tiet kiem nang luong */
                    /* (co the them ham de dua MCU vao che do sleep) */ 
                }
                
                /* Doi dieu kien tot hon trong 10 giay */
                if (GetMillis() - lowPowerStateStartTime >= 10000) {
                    systemState = SYSTEM_STATE_INIT;
                    lowPowerStateFirstEntry = 1; /* Reset flag */
                }
                break;
                
            default:
                systemState = SYSTEM_STATE_INIT;
                break;
        }
        
        /* Non-blocking delay tuy vao trang thai */
        if (systemState == SYSTEM_STATE_RUNNING) {
            DelayMs(50); /* Cap nhat nhanh hon trong che do chay binh thuong */
        } else if (systemState == SYSTEM_STATE_INIT) {
            DelayMs(100); /* Delay ngan cho INIT state */
        } else {
            /* ERROR va LOW_POWER states: delay ngan */
            DelayMs(10); /* Delay ngan de he thong responsive */
        }
    }

}
