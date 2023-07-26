/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: July 8, 2023
 *      Author: Ashish Sharma
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"
/*
 * Peripheral Clock Setup
 */
/***************************************************************************************

 * @Function		:GPIO_PeriClockControl
 * @Brief			:This function enables or disables peripheral clock for the given GPIO port
 * @Parameter[1]	:pGPIOx = Base address of the GPIO peripheral
 * @Parameter[2]	:EnorDi = ENABLE or DISABLE macros
 *
 * @Return			:none
 * @Note			:none

 **************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}
	else if (EnorDi == DISABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}
			else if (pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}
		}

}

/*
 * GPIO Initialization and De-Inintialization
 */

/***************************************************************************************

 * @Function		:
 * @Brief			:
 * @Parameter[1]	:
 * @Parameter[2]	:
 *
 * @Return			:
 * @Note			:

 **************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//1. Configure the mode of GPIO pin
	uint32_t temp = 0;		//temporary register

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Non Interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Clearing the bit
		pGPIOHandle->pGPIOx->MODER |= temp;			// Setting the bit

	}
	else
	{
		//Interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the FTSR
			EXTI->FTSR |= (1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			EXTI->RTSR |= (1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}

		//2.Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);


		//3. Enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	}

	temp = 0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Clearing the bit
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;			//Setting the bit

	temp =0;

	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Clearing the bit
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp =0;

	//4. Configure the Output Type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~ (0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Clearing the bit
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp =0;

	//5. Configure the Alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure alternate function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;		//AFR[0] alternate function low register and AFR[1] high register will be selected in the temp1.
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;		//Bit position is selected in the temp2 variable.
		pGPIOHandle->pGPIOx->AF[temp1] &= ~ (0xF << (4 * temp2));	//Clearing the bit
		pGPIOHandle->pGPIOx->AF[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));		//4 bits are used in AF register and so multiplied by 4.

	}
}

/***************************************************************************************

 * @Function		:
 * @Brief			:
 * @Parameter[1]	:
 * @Parameter[2]	:
 *
 * @Return			:
 * @Note			:

 **************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
}

/*
 * Data Read and Write
 */

/***************************************************************************************

 * @Function		:GPIO_ReadFromInputPin
 * @Brief			:These registers contain the input value of the corresponding I/O port.
Here we are shifting the value of bit by the Pin number position and then masking it. For example
if we want to read from 5th bit then Pin Number will be 5 and we move the bit to 5 times and then
read the very first bit.

 * @Parameter[1]	:pGPIOx
 * @Parameter[2]	:PinNumber
 *
 * @Return			: value, either 0 or 1
 * @Note			:

 **************************************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value =  (uint8_t) ((pGPIOx->IDR  << PinNumber) & 0x00000001);
	return value;
}

/***************************************************************************************

 * @Function		:GPIO_ReadFromInputPort
 * @Brief			:
 * @Parameter[1]	:pGPIOx
 * @Parameter[2]	: none
 *
 * @Return			:
 * @Note			:

 **************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		//Write 1 to the output data register at the bit field corresponding to the Pin Number
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		//Write 0
		pGPIOx->ODR &= ~(1<<PinNumber);

	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1<<PinNumber);						// Can also write this: pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));

		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber%32));

		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber%64));

		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//used (8 - NO_PR_BITS_IMPLEMENTED) because in the 8 bit register only upper half bits are implemented and the lower half is not used
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << ((8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED)));
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//To clear the bit we have to set it in PR
		EXTI->PR |= (1 << PinNumber);
	}
}
