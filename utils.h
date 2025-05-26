/**
 * @file utils.h
 * @brief Thu vien cac ham tien ich
 */

#ifndef __UTILS_H
#define __UTILS_H

#include "stm32f10x.h"

/**
 * @brief Khoi tao timer he thong (SysTick)
 */
void TimerInit(void);

/**
 * @brief Lay so milli giay tu khi khoi dong
 * @return So milli giay
 */
uint32_t GetMillis(void);

/**
 * @brief Lay so micro giay tu khi khoi dong
 * @return So micro giay
 */
uint32_t GetMicros(void);

/**
 * @brief Ham delay milli giay su dung SysTick
 * @param ms So milli giay can delay
 */
void DelayMs(uint32_t ms);

/**
 * @brief Ham delay micro giay su dung SysTick
 * @param us So micro giay can delay
 */
void DelayUs(uint32_t us);

/**
 * @brief Khoi tao Timer2
 */
void Timer_Init(void);

/**
 * @brief Delay 1ms su dung Timer2
 */
void Delay_1ms(void);

/**
 * @brief Delay voi so ms chi dinh su dung Timer2
 * @param time So ms can delay
 */
void Delay_ms(u16 time);
void IWDG_Init(uint32_t timeout_ms);
void IWDG_Refresh(void);
void Delay_Precise_Ms(uint32_t u32Delay);
#endif /* __UTILS_H */
