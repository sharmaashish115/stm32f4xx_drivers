/*
 *MCU specific Header file
 * stm32f407xx.h
 *
 *  Created on: July 5, 2023
 *      Author: Ashish Sharma
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/******************START : Processor Specific Details*******************
 *
 * ARM-Cortex M4 Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)

/*
 * ARM-Cortex M4 Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0				((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0xE000E188)

/*
 * ARM-Cortex M4 Processor NVIC Priority register Addresses
 */
#define NVIC_PR_BASE_ADDR		((volatile uint32_t*)0XE000E400)

#define NO_PR_BITS_IMPLEMENTED		4 	// Number of priority bits implemented
/*
 * Base addresses of Flash and SRAM addresses
 */
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 *
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 *
 */
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define UART7_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR			(APB1PERIPH_BASEADDR + 0x7C00)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

/************* Peripheral Register Definition Structure ***********/
/*
 * Registers of a peripheral are specific to Micro-controller
 */
typedef struct
{
	volatile uint32_t MODER;		//GPIO Port Mode Register | Address offset: 0x00
	volatile uint32_t OTYPER;		//GPIO port output type register | Address offset: 0x04
	volatile uint32_t OSPEEDR;		//GPIO port output speed register | Address offset: 0x08
	volatile uint32_t PUPDR;		//GPIO port pull-up/pull-down register | Address offset: 0x0C
	volatile uint32_t IDR;			//GPIO port input data register | Address offset: 0x10
	volatile uint32_t ODR;			//GPIO port output data register | Address offset: 0x14
	volatile uint32_t BSRR;			//GPIO port bit set/reset register | Address offset: 0x18
	volatile uint32_t LCKR;			//GPIO port configuration lock register | Address offset: 0x1C
	volatile uint32_t AF[2];		//GPIO AFR[0] alternate function low register and 
						//	AFR[1] alternate function high register | Address offset: 0x20 and 0x24
}GPIO_RegDef_t;

//Example of typecasting
//GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*)0x40020000; // Base address of GPIOA = 0x40020000

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	volatile uint32_t CR;			//RCC clock control register | Address offset: 0x00
	volatile uint32_t PLLCFGR;		//RCC PLL configuration register | Address offset: 0x04
	volatile uint32_t CFGR;			//RCC clock configuration register | Address offset: 0x08
	volatile uint32_t CIR;			//RCC clock interrupt register | Address offset: 0x0C
	volatile uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register | Address offset: 0x10
	volatile uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register | Address offset: 0x14
	volatile uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register | Address offset: 0x18
	volatile uint32_t RESERVED0;	//RESERVED
	volatile uint32_t APB1RSTR;		//RCC APB1 peripheral reset register | Address offset: 0x20
	volatile uint32_t APB2RSTR;		//RCC APB2 peripheral reset register | Address offset: 0x24
	volatile uint32_t RESERVED1[2];	//RESERVED
	volatile uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register | Address offset: 0x30
	volatile uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register | Address offset: 0x34
	volatile uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable register | Address offset: 0x38
	volatile uint32_t RESERVED2;	//RESERVED
	volatile uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register | Address offset: 0x40
	volatile uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register | Address offset: 0x44
	volatile uint32_t RESERVED3[2];	//RESERVED
	volatile uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register | Address offset: 0x50
	volatile uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register | Address offset: 0x54
	volatile uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register | Address offset: 0x58
	volatile uint32_t RESERVED4;	//RESERVED
	volatile uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register | Address offset: 0x60
	volatile uint32_t APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register
	volatile uint32_t RESERVED5[2];	//RESERVED
	volatile uint32_t BDCR;			//RCC Backup domain control register | Address offset: 0x70
	volatile uint32_t CSR;			//RCC clock control & status register | Address offset: 0x74
	volatile uint32_t RESERVED6[2];	//RESERVED
	volatile uint32_t SSCGR;		//RCC spread spectrum clock generation register | Address offset: 0x80
	volatile uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register | Address offset: 0x84

	volatile uint32_t PLLSAICFGR;	//			,,	| Address offset: 0x88
	volatile uint32_t DCKCFGR;		//			,,	| Address offset: 0x8C
	volatile uint32_t CKGATENR;		//			,,	| Address offset: 0x90
	volatile uint32_t DCKCFGR2;		//			,,	| Address offset: 0x94

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	volatile uint32_t IMR;			//EXTI register, Interrupt mask register, Address offset: 0x00
	volatile uint32_t EMR;			//EXTI register, Event mask register, Address offset: 0x04
	volatile uint32_t RTSR;			// ,,	,,		, Rising trigger selection register, Address offset: 0x08
	volatile uint32_t FTSR;			// ,,	,,		, Falling trigger selection register, Address offset: 0x0C
	volatile uint32_t SWIER;		// ,,	,,		, Software interrupt event register, Address offset: 0x10
	volatile uint32_t PR;			// ,,	,,		, Pending register, Address offset: 0x14

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for
 */
typedef struct
{
	volatile uint32_t MEMRMP;		//SYSCFG register, SYSCFG memory remap register, offset: 0x00
	volatile uint32_t PMC;			//SYSCFG peripheral mode configuration register, offset: 0x04

	volatile uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register 1, offset: 0x08 - 0x14
	volatile uint32_t CMPCR;		//Compensation cell control register, Address offset: 0x20

}SYSCFG_RegDef_t;

/*
 * Peripheral Definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR) |= (1<<0)
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR) |= (1<<1)
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR) |= (1<<2)
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR) |= (1<<3)
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR) |= (1<<4)
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR) |= (1<<5)
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR) |= (1<<6)
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR) |= (1<<7)
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR) |= (1<<8)

/*
 * Clock Enable Macros for I2C peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR) |= (1<<21)
#define I2C2_PCLK_EN()		(RCC->APB1ENR) |= (1<<22)
#define I2C3_PCLK_EN()		(RCC->APB1ENR) |= (1<<23)

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI2_PCLK_EN()		(RCC->APB1ENR) |= (1<<14)
#define SPI3_PCLK_EN()		(RCC->APB1ENR) |= (1<<15)

#define SPI1_PCLK_EN()		(RCC->APB2ENR) |= (1<<12)
#define SPI4_PCLK_EN()		(RCC->APB2ENR) |= (1<<13)
#define SPI5_PCLK_EN()		(RCC->APB2ENR) |= (1<<20)
#define SPI6_PCLK_EN()		(RCC->APB2ENR) |= (1<<21)

/*
 * Clock enable macros for USARTx peripherals
 */
#define USART2_PCLK_EN()	(RCC->APB1ENR) |= (1<<17)
#define USART3_PCLK_EN()	(RCC->APB1ENR) |= (1<<18)
#define UART4_PCLK_EN()		(RCC->APB1ENR) |= (1<<19)
#define UART5_PCLK_EN()		(RCC->APB1ENR) |= (1<<20)

/*
 * Clock enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR) |= (1<<14)

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<0)
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<1)
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<2)
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<3)
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<4)
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<5)
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<6)
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<7)
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR) &= ~(1<<8)

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR) &= ~(1<<21)
#define I2C2_PCLK_DI()		(RCC->APB1ENR) &= ~(1<<22)
#define I2C3_PCLK_DI()		(RCC->APB1ENR) &= ~(1<<23)

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI2_PCLK_DI()		(RCC->APB1ENR) &= ~(1<<14)
#define SPI3_PCLK_DI()		(RCC->APB1ENR) &= ~(1<<15)

#define SPI1_PCLK_DI()		(RCC->APB2ENR) &= ~(1<<12)
#define SPI4_PCLK_DI()		(RCC->APB2ENR) &= ~(1<<13)
#define SPI5_PCLK_DI()		(RCC->APB2ENR) &= ~(1<<20)
#define SPI6_PCLK_DI()		(RCC->APB2ENR) &= ~(1<<21)

/*
 * Clock Disable macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR) &= ~(1<<14)


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do {RCC->AHB1RSTR |= (1<<0);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOB_REG_RESET()	do {RCC->AHB1RSTR |= (1<<1);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOC_REG_RESET()	do {RCC->AHB1RSTR |= (1<<2);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOD_REG_RESET()	do {RCC->AHB1RSTR |= (1<<3);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOE_REG_RESET()	do {RCC->AHB1RSTR |= (1<<4);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOF_REG_RESET()	do {RCC->AHB1RSTR |= (1<<5);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOG_REG_RESET()	do {RCC->AHB1RSTR |= (1<<6);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOH_REG_RESET()	do {RCC->AHB1RSTR |= (1<<7);	RCC->AHB1RSTR &= ~(1<<0);} while(0)
#define GPIOI_REG_RESET()	do {RCC->AHB1RSTR |= (1<<8);	RCC->AHB1RSTR &= ~(1<<0);} while(0)

#define GPIO_BASEADDR_TO_CODE(x)		  ( (x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOF) ? 5 :\
											(x == GPIOG) ? 6 :\
											(x == GPIOH) ? 7 :\
											(x == GPIOI) ? 8 :0 )

/*
 * IRQ(Interrupt Request) Numbers of STM32F407xx MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define ENABLE 			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
