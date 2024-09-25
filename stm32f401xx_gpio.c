/*
 * stm32f401xx_gpio.c
 *
 *  Created on: Jul 27, 2024
 *      Author: KURAPIKA
 */
#include "stm32f401xx_gpio.h"

/****************************
 * @fn				- GPIO_PClkControl
 *
 * @brief			- Enable or Disable GPIO Peripheral Clock
 * @param[in]		- GPIO Base Address
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 */

void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN(); //it's a c Macros
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
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
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
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}
/****************************
 * @fn				- GPIO_Init
 *
 * @brief			- INIT GPIO
 * @param[in]		- pointer to GPIO_handle_t structure
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; // temp register
	//1. configure the mode of the GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clearing the RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t temp1 = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
		uint8_t temp2 = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 );

		{
			if(pGPIOHandle->pGPIOx == GPIOA)
			{
				SYSCFG->EXTICR[temp1] |= (0x0 << 4 * temp2 );
			}
			if(pGPIOHandle->pGPIOx == GPIOB)
			{
				SYSCFG->EXTICR[temp1] |= (0x1 << 4 * temp2 );
			}
			if(pGPIOHandle->pGPIOx == GPIOC)
			{
				SYSCFG->EXTICR[temp1] |= (0x2 << 4 * temp2 );
			}
			if(pGPIOHandle->pGPIOx == GPIOD)
			{
				SYSCFG->EXTICR[temp1] |= (0x3 << 4 * temp2 );
			}
			if(pGPIOHandle->pGPIOx == GPIOE)
			{
				SYSCFG->EXTICR[temp1] |= (0x4 << 4 * temp2 );
			}
			if(pGPIOHandle->pGPIOx == GPIOH)
			{
				SYSCFG->EXTICR[temp1] |= (0x7 << 4 * temp2 );
			}
		}

		//3. Enable the EXTI interrupt delivery using Interrupt mask register
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;
	//3. configure the pull up pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControle << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;
	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	temp=0;
	//5. configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) > 7)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8));
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0x7 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->AFR[1] |= temp;
		}
		else
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->AFR[0] |= temp;
		}
	}
	temp=0;

}
/****************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- de-INIT GPIO
 * @param[in]		- pointer to GPIO_RegDef_t structure
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- we need to use RCC register
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET(); // you have to set the bit field then reset (1 --> 0)
	}
	if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}


}
/*
 * Data Read and Write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber )
{

		uint8_t value;
		value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
		return value;

}
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR) ;
	return value;
}
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
		(pGPIOx->ODR) |= (1 << PinNumber);//write 1

	}
	else
	{
		(pGPIOx->ODR) &= ~(1 << PinNumber);//write 0

	}

}
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

	//(pGPIOx->ODR) &= ~(value);
	(pGPIOx->ODR) = value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	(pGPIOx->ODR) ^= (1 << PinNumber);

}
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfiq(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) // configure the IRQ number of GPIO PIN (Enable/Set priority)
{
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;
	uint8_t temp_IPR1 = IRQNumber / 4;
	uint8_t temp_IPR2 = IRQNumber % 4;
	uint8_t shifted_amount = (8* temp_IPR2)+(8-NO_PR_BITS_IMPLEMENTED); // +4 because only the 4 upper bit are implemented

	NVIC_IPR->IPR[temp_IPR1] &= ~(0xFF << (8* temp_IPR2));
	NVIC_IPR->IPR[temp_IPR1] |= (IRQPriority << shifted_amount);

	if ( EnorDi == ENABLE)
	{
		NVIC_ISER->ISER[temp1] |= (1 << temp2);

	}
	else
	{
		NVIC_ICER->ICER[temp1] |= (1 << temp2);
	}


}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1<<PinNumber))
	{
	EXTI->PR |= (1 << PinNumber);
	}
}




