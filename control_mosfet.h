/**
 * @file control_mosfet.h
 * @brief Thu vien bat tat mosfet dieu khien sac-xa
 */

#ifndef __CONTROL_MOSFET_H
#define __CONTROL_MOSFET_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void InitControlMosfet(void);
void setMosfet(uint8_t Pin_Charge_0, uint8_t Pin_Charge_1, uint8_t Pin_DisCharge, uint8_t Pin_Dc, uint8_t Pin_Load );

#endif /* __MPPT_H */
