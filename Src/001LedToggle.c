/**
 ******************************************************************************
 * @file           : main.c
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

void delay (void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed;
    memset(&gpioLed,0,sizeof(gpioLed));


    gpioLed.pGPIOx = GPIOD;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = 15;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&gpioLed);

    while (1)
    {
    	GPIO_ToggleOutputPin(GPIOD, 15);
    	delay();
    }



	return 0;
}
