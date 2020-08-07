/*
 * stm32f44xx_gpio_driver.c
 *
 *  Created on: 04-Aug-2020
 *      Author: prasanna
 */

#include "stm32f446xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
				}
				else if (pGPIOx == GPIOB){
					GPIOB_PCLK_DI();
				}
				else if (pGPIOx == GPIOC){
					GPIOC_PCLK_DI();
				}
				else if (pGPIOx == GPIOD){
					GPIOD_PCLK_DI();
				}
				else if (pGPIOx == GPIOE){
					GPIOE_PCLK_DI();
				}
				else if (pGPIOx == GPIOF){
					GPIOF_PCLK_DI();
				}
				else if (pGPIOx == GPIOG){
					GPIOG_PCLK_DI();
				}
				else if (pGPIOx == GPIOH){
					GPIOH_PCLK_DI();
				}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE)
{

	uint32_t temp = 0;

	if (pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinMode){
		temp = pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinMode << (2 *pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
		pGPIOHANDLE->pGPIOx->MODER &= ~(0x3 << pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
		pGPIOHANDLE->pGPIOx->MODER |= temp;
	}
	else {

	}

	temp=0;

	temp = pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinSpeed << (2 *pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHANDLE->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHANDLE->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	temp = pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinPuPdControl << (2 *pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHANDLE->pGPIOx->PUPDR &= ~(0x3 << pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHANDLE->pGPIOx->PUPDR |= temp;

	temp = 0;

	temp = pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinOPType << pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber;
	pGPIOHANDLE->pGPIOx->OTYPER &= ~(0x1 << pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber);
	pGPIOHANDLE->pGPIOx->OTYPER |= temp;

	if (pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinMode	== GPIO_MODE_ALTFN){

		uint8_t temp1,temp2;

		temp1 = pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber / 8;
		temp2 = pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinNumber % 8;
		pGPIOHANDLE->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHANDLE->pGPIOx->AFR[temp1] |= (pGPIOHANDLE->GPIO_PinConfig_t.GPIO_PinAltFunMode << (4 * temp2));

	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA){
				GPIOA_REG_RESET();
			}
			else if (pGPIOx == GPIOB){
				GPIOB_REG_RESET();
			}
			else if (pGPIOx == GPIOC){
				GPIOC_REG_RESET();
			}
			else if (pGPIOx == GPIOD){
				GPIOD_REG_RESET();
			}
			else if (pGPIOx == GPIOE){
				GPIOE_REG_RESET();
			}
			else if (pGPIOx == GPIOF){
				GPIOF_REG_RESET();
			}
			else if (pGPIOx == GPIOG){
				GPIOG_REG_RESET();
			}
			else if (pGPIOx == GPIOH){
				GPIOH_REG_RESET();
			}

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	uint16_t value;

	value = (uint16_t)pGPIOx-> IDR;

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value)
{

	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

	pGPIOx->ODR = Value;

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{

	pGPIOx->ODR ^= ( 1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_IRQHandling(uint8_t PinNumber){

}
