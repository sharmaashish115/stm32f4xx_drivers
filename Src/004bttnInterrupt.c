/*
 * 004bttnInterrupt.c
 *
 *  Created on: Jul 25, 2023
 *      Author: Ashish Sharma
 */

/**
 ******************************************************************************
 * @file           : 003ExtLedBttn.c
 * Created on      : July 12, 2023
 * @author         : Ashish Sharma
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "stm32f407xx.h"

#include <stdint.h>
#include <string.h>


void delay (void)
{
	for(uint32_t i = 0; i < 50000/2; i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed, gpioBtn;

    memset(&gpioLed,0,sizeof(gpioLed));
    memset(&gpioBtn,0,sizeof(gpioLed));

    //This is LED configuration
    gpioLed.pGPIOx = GPIOD;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&gpioLed);

    //This is Button configuration
    gpioBtn.pGPIOx = GPIOD;
    gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioBtn);

	//IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);

}

void EXTI9_5_IRQHandler(void)
{
	delay();

	//handle the interrupt
	GPIO_IRQHandling(5);
	GPIO_ToggleOutputPin(GPIOD, 12);

}

