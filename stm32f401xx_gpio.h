/*
 * stm32f401xx_gpio.h
 *
 *  Created on: Jul 27, 2024
 *      Author: KURAPIKA
 */

#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_

#include "stm32f401xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber; //@GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode; // possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;// possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControle;// possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;// possible values from @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* This is a Handle structure for a GPIO pin */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; // this holds the base @ of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // this holds GPIO pin configuration setting
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
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
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6
/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0 //push pull
#define GPIO_OP_TYPE_OD		1 //open drain
/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3
/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		    0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

//Driver APIs
/*
 * Peripheral Clock Setup
 */
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Data Read and Write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber );
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfiq(uint8_t IRGNumber, uint8_t IRGPriority, uint8_t EnorDi); // configure the IRQ number of GPIO PIN (Enable/Set priority)
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F401XX_GPIO_H_ */


