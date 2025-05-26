/**
 * @file mppt.c
 * @brief Thu vien bat tat mosfet dieu khien sac-xa
 */

#include "control_mosfet.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void InitControlMosfet(void)
{
    GPIO_InitTypeDef GPIO_Control_Mosfet;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_Control_Mosfet.GPIO_Pin = GPIO_Pin_11;
    GPIO_Control_Mosfet.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Control_Mosfet.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Control_Mosfet);

    GPIO_Control_Mosfet.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_13;
    GPIO_Control_Mosfet.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Control_Mosfet.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_Control_Mosfet);
}

void setMosfet(uint8_t Pin_Charge_0, uint8_t Pin_Charge_1, uint8_t Pin_DisCharge, uint8_t Pin_Dc, uint8_t Pin_Load ) /* PB0 - PB1 - PA11 - PB3 - PB13 */
{
	if (Pin_Charge_0)  GPIO_SetBits(GPIOB, GPIO_Pin_0);
	else  						 GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	if (Pin_Charge_1)  GPIO_SetBits(GPIOB, GPIO_Pin_1);
	else  						 GPIO_ResetBits(GPIOB, GPIO_Pin_1);	
	if (Pin_DisCharge) GPIO_SetBits(GPIOA, GPIO_Pin_11);
	else  						 GPIO_ResetBits(GPIOA, GPIO_Pin_11);	
	if (Pin_Dc)    		 GPIO_SetBits(GPIOB, GPIO_Pin_3);
	else  						 GPIO_ResetBits(GPIOB, GPIO_Pin_3);	
	if (Pin_Load) 		 GPIO_SetBits(GPIOB, GPIO_Pin_13);
	else  						 GPIO_ResetBits(GPIOB, GPIO_Pin_13);	
}
