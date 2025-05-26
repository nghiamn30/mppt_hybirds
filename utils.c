/**
 * @file utils.c
 * @brief Thuc thi thu vien tien ich - FIXED VERSION
 */

#include "utils.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

static volatile uint32_t msTicks = 0;

/**
 * @brief Handler cho SysTick interrupt
 */
void SysTick_Handler(void) {
    msTicks++;
}

/**
 * @brief Khoi tao timer he thong
 */
void TimerInit(void) {
    /* Cau hinh SysTick interrupt moi 1ms */
    SysTick_Config(SystemCoreClock / 1000);
}

/**
 * @brief Lay so milli giay tu khi khoi dong
 * @return So milli giay
 */
uint32_t GetMillis(void) {
    return msTicks;
}

/**
 * @brief Lay so micro giay tu khi khoi dong
 * @return So micro giay
 */
uint32_t GetMicros(void) {
    /* Tinh so micro giay tu msTicks va SysTick->VAL */
    uint32_t micros = msTicks * 1000 + (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock / 1000000);
    return micros;
}

/**
 * @brief Khoi tao Timer2 cho delay functions
 */
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef timer_init;
    
    /* Bat clock cho Timer2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* Cau hinh Timer2 */
    timer_init.TIM_CounterMode = TIM_CounterMode_Up;
    timer_init.TIM_Period = 65535;          /* Gia tri toi da cua counter */
    timer_init.TIM_Prescaler = 72 - 1;      /* 72MHz / 72 = 1MHz (1us per tick) */
    timer_init.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_init.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIM2, &timer_init);
    TIM_Cmd(TIM2, ENABLE); /* Enable Timer2 */
}

/**
 * @brief Delay 1ms su dung Timer2
 */
void Delay_1ms(void)
{
    /* Reset counter */
    TIM_SetCounter(TIM2, 0);
    
    /* Doi den khi counter dat gia tri 1000 = 1ms (1MHz clock) */
    while(TIM_GetCounter(TIM2) < 1000);
}

/**
 * @brief Delay voi so ms chi dinh su dung Timer2
 * @param time So ms can delay
 */
void Delay_ms(u16 time)
{
    /* Khoi tao Timer2 neu chua khoi tao */
    static uint8_t timer_initialized = 0;
    if (!timer_initialized) {
        Timer_Init();
        timer_initialized = 1;
    }
    
    while(time--)
    {
        Delay_1ms();
    }
}

/**
 * @brief Ham delay milli giay su dung SysTick - NON-BLOCKING
 * @param ms So milli giay can delay
 */
void DelayMs(uint32_t ms) {
    uint32_t startTicks = msTicks;
    while ((msTicks - startTicks) < ms);
}

/**
 * @brief Ham delay micro giay su dung SysTick
 * @param us So micro giay can delay
 */
void DelayUs(uint32_t us) {
    uint32_t startMicros = GetMicros();
    while ((GetMicros() - startMicros) < us);
}

/**
 * @brief FIXED: Delay precise su dung Timer2 thay vi SysTick
 * @param u32Delay So ms can delay
 */
void Delay_Precise_Ms(uint32_t u32Delay)
{
    /* Su dung DelayMs() thay vi truc tiep manipulate SysTick */
    /* Cach nay khong conflict voi GetMillis() */
    DelayMs(u32Delay);
}

/**
 * @brief ALTERNATIVE: Delay precise su dung Timer2 hardware
 * @param u32Delay So ms can delay
 */
void Delay_Precise_Ms_Timer2(uint32_t u32Delay)
{
    /* Khoi tao Timer2 neu chua khoi tao */
    static uint8_t timer_initialized = 0;
    if (!timer_initialized) {
        Timer_Init();
        timer_initialized = 1;
    }
    
    /* Su dung Timer2 thay vi SysTick */
    while(u32Delay) 
    {
        /* Reset counter */
        TIM_SetCounter(TIM2, 0);
        
        /* Doi 1ms (1000 ticks voi 1MHz clock) */
        while(TIM_GetCounter(TIM2) < 1000);
        
        --u32Delay;
    }
}

/**
 * @brief Backup va restore SysTick configuration (advanced)
 * @param u32Delay So ms can delay
 */
void Delay_Precise_Ms_Safe(uint32_t u32Delay)
{
    /* Backup SysTick configuration */
    uint32_t backup_LOAD = SysTick->LOAD;
    uint32_t backup_VAL = SysTick->VAL;
    uint32_t backup_CTRL = SysTick->CTRL;
    
    /* Su dung SysTick cho delay */
    while(u32Delay) 
    {
        SysTick->LOAD = 72U * 1000U - 1U;
        SysTick->VAL = 0U;
        SysTick->CTRL = 5U; /* Disable interrupt, enable counter */
        
        while (!(SysTick->CTRL & (1U << 16U)))
        {
            /* Wait for countdown */
        }
        --u32Delay;
    }
    
    /* Restore SysTick configuration */
    SysTick->LOAD = backup_LOAD;
    SysTick->VAL = backup_VAL;
    SysTick->CTRL = backup_CTRL;
}

/* IWDG functions - removed as per requirement */
/* These functions are commented out since watchdog is disabled */

/*
void IWDG_Init(uint32_t timeout_ms)
{
    // Implementation removed - watchdog disabled
}

void IWDG_Refresh(void)
{
    // Implementation removed - watchdog disabled  
}
*/
