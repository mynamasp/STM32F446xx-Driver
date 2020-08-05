/*
 * stm32f446xx.h
 *
 *  Created on: Aug 3, 2020
 *      Author: prasanna
 */
#include "stdint.h"

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000UL
#define SRAM1_BASEADDR			0x20000000UL
#define ROM						0x1FFF0000UL
#define SRAM2					0x2001C000UL
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR         0x40000000UL
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000UL
#define AHB1PERIPH_BASEADDR		0x40020000UL
#define AHB2PERIPH_BASEADDR		0x50000000UL
#define AHB3PERIPH_BASEADDR		0x60000000UL
/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3800)

#define	GPIOA_BASEADDR			0x40020000UL
#define	GPIOB_BASEADDR			0x40020400UL
#define	GPIOC_BASEADDR			0x40020800UL
#define	GPIOD_BASEADDR			0X40020C00UL
#define	GPIOE_BASEADDR			0x40021000UL
#define	GPIOF_BASEADDR			0x40021400UL
#define	GPIOG_BASEADDR			0x40021800UL
#define	GPIOH_BASEADDR			0x40021C00UL

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR+0x5C00)

#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR+0x3C00)

#define USART2_BASEADDR         (APB1PERIPH_BASEADDR+0x4400)
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR+0x4800)
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR+0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR+0x3C00)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR+0x1400)
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR+0x3000)
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR+0x3400)
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR+0x3800)

//some generic macros

#define ENABLE 					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET


#define _vo volatile


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct{
	_vo uint32_t	MODER;
	_vo uint32_t	OTYPER;
	_vo uint32_t	OSPEEDR;
	_vo uint32_t	PUPDR;
	_vo uint32_t	IDR;
	_vo uint32_t	ODR;
	_vo uint32_t	BSRR;
	_vo uint32_t	LCKR;
	_vo uint32_t	AFR[2];
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct{
	_vo uint32_t	CR;
	_vo uint32_t	PLLCFGR;
	_vo uint32_t	CFGR;
	_vo uint32_t	CIR;
	_vo uint32_t	AHB1RSTR;
	_vo uint32_t	AHB2RSTR;
	_vo uint32_t	AHB3RSTR;
		uint32_t	RESERVED0;
	_vo uint32_t	APB1RSTR;
	_vo uint32_t	APB2RSTR;
		uint32_t	RESERVED1[2];
	_vo uint32_t	AHB1ENR;
	_vo uint32_t	AHB2ENR;
	_vo uint32_t	AHB3ENR;
		uint32_t	RESERVED2;
	_vo uint32_t	APB1ENR;
	_vo uint32_t	APB2ENR;
		uint32_t	RESERVED3[2];
	_vo uint32_t	AHB1LPENR;
	_vo uint32_t	AHB2LPENR;
	_vo uint32_t	AHB3LPENR;
		uint32_t	RESERVED4;
	_vo uint32_t	APB1LPENR;
	_vo uint32_t	APB2LPENR;
		uint32_t	RESERVED5[2];
	_vo uint32_t	BDCR;
	_vo uint32_t	CSR;
		uint32_t	RESERVED6[2];
	_vo uint32_t	SSCGR;
	_vo uint32_t	PLLI2SCFGR;
	_vo uint32_t	PLLSAICFGR;
	_vo uint32_t	DCKCFGR;
	_vo uint32_t	CKGATENR;
	_vo uint32_t	DCKCFGR2;

}RCC_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */


#define GPIOA_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 0))
#define GPIOB_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 1))
#define GPIOC_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 2))
#define GPIOD_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 3))
#define GPIOE_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 4))
#define GPIOF_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 5))
#define GPIOG_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 6))
#define GPIOH_PCLK_EN()	 ( RCC->AHB1ENR |= ( 1 << 7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 23))

/*
 * Clock Enable Macros for SPIx peripheralsbu
 */

#define SPI1_PCLK_EN()	 ( RCC->APB2ENR |= ( 1 << 12))
#define SPI2_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()	 ( RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_EN()	 ( RCC->APB2ENR |= ( 1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PLCK_EN() ( RCC->APB2ENR |= ( 1 << 4))
#define USART2_PLCK_EN() ( RCC->APB1ENR |= ( 1 << 17))
#define USART3_PLCK_EN() ( RCC->APB1ENR |= ( 1 << 18))
#define UART4_PLCK_EN()  ( RCC->APB1ENR |= ( 1 << 19))
#define UART5_PLCK_EN()  ( RCC->APB1ENR |= ( 1 << 20))
#define USART6_PLCK_EN() ( RCC->APB2ENR |= ( 1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PLCK_EN() ( RCC->APB2ENR |= ( 1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 0))
#define GPIOB_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 1))
#define GPIOC_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 2))
#define GPIOD_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 3))
#define GPIOE_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 4))
#define GPIOF_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 5))
#define GPIOG_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 6))
#define GPIOH_PCLK_DI()	 ( RCC->AHB1ENR &= ~( 1 << 7))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()	 ( RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI()	 ( RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI()	 ( RCC->APB1ENR &= ~( 1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()	 ( RCC->APB2ENR &= ~( 1 << 12))
#define SPI2_PCLK_DI()	 ( RCC->APB1ENR &= ~( 1 << 14))
#define SPI3_PCLK_DI()	 ( RCC->APB1ENR &= ~( 1 << 15))
#define SPI4_PCLK_DI()	 ( RCC->APB2ENR &= ~( 1 << 13))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PLCK_DI() ( RCC->APB2ENR &= ~( 1 << 4))
#define USART2_PLCK_DI() ( RCC->APB1ENR &= ~( 1 << 17))
#define USART3_PLCK_DI() ( RCC->APB1ENR &= ~( 1 << 18))
#define UART4_PLCK_DI()  ( RCC->APB1ENR &= ~( 1 << 19))
#define UART5_PLCK_DI()  ( RCC->APB1ENR &= ~( 1 << 20))
#define USART6_PLCK_DI() ( RCC->APB2ENR &= ~( 1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PLCK_DI() ( RCC->APB2ENR &= ~( 1 << 14))


#endif /* INC_STM32F446XX_H_ */
