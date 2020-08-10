/*
 * stm32f44xx_gpio_driver.c
 *
 *  Created on: 04-Aug-2020
 *      Author: prasanna
 */
#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t	GPIO_PinOPType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig_t;
}GPIO_Handle_t;


/*
 * GPIO Possible Pin Numbers
 */

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_1O		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * GPIO Possible Pin States
 */

#define HIGH				1
#define LOW 				0


/*
 * GPIO Possible Pin Modes
 */

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6
#define OUTPUT				1
#define INPUT 				0
#define ALTFN				2
#define ANALOG				3
#define IT_FT				4
#define IT_RT				5
#define IT_RFT				6


/*
 * GPIO Possible Pin Output Type
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1
#define PP					0
#define OD					1
#define NONE				2

/*
 * GPIO Possible Pin Output Speed
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_FULL		3
#define LOW					0
#define MEDIUM				1
#define FAST				2
#define FULL				3


/*
 * GPIO Pin Pull-up And Pull Down Configuration	Macros
 */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
#define NO_PUPD				0
#define PULLUP				1
#define PUSHDOWN			2

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
void GPIO_PinOutput(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t PinSpeed,uint8_t PinOPType,uint8_t PinPUPDC);
void GPIO_PinInput(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t PinSpeed,uint8_t PinPUPDC);


void GPIO_PinSetup(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t PinMode,uint8_t PinSpeed,uint8_t PinOPType,uint8_t PinPUPDC);


/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
