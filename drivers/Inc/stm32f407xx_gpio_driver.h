/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: July 8, 2023
 *      Author: Ashish Sharma
 */
#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO Pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			//Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			//Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	//Possible values from @Pin PU & PD Config
	uint8_t GPIO_PinOPType;			//Possible values from @Pin Output Types
	uint8_t GPIO_PinAltFunMode;		//Possible values from
}GPIO_PinConfig_t;

/*
 * This is a Handle Structure for a GPIO Pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;		//This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//This holds GPIO pin configuration settings.

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12 		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin possible modes
 */
#define GPIO_MODE_IN		0		//00: Input (reset state)
#define	GPIO_MODE_OUT		1		//01: General purpose output mode
#define GPIO_MODE_ALTFN		2		//10: Alternate function mode
#define GPIO_MODE_ANALOG	3		//11: Analog mode

#define GPIO_MODE_IT_FT		4		// IT->Input ; FT-> Falling Edge Trigger ;
#define GPIO_MODE_IT_RT		5		// RT-> Rising Edge Trigger
#define GPIO_MODE_IT_RFT	6		// RFT-> Rising Falling Edge Trigger

/*
 *
 *@Pin Output Types
 * GPIO Pin possible Output Types
 */
#define GPIO_OP_TYPE_PP		0		//PP-> 0: Output push-pull (reset state)
#define GPIO_OP_TYPE_OD		1		//OD-> 1: Output open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin possible Output Speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERYHIGH		3

/*
 * @Pin PU & PD Config
 * GPIO Pin Pull up and Pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * *********API Supported by this driver**********
 * 		 Check the function definitions
 */
/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Initialization and De-Inintialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
