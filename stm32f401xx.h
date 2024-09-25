/*
 * stm32f401xx.h
 *
 *  Created on: Jul 26, 2024
 *      Author: KURAPIKA
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_
#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))
#define ENABLE 		1
#define DISABLE 	0
#define SET			ENABLE
#define	RESET 		DISABLE
#define	GPIO_PIN_SET 		ENABLE
#define	GPIO_PIN_RESET 		DISABLE
#define NO_PR_BITS_IMPLEMENTED 4
#define FLAG_RESET		RESET
#define FLAG_SET		SET
#define I2C_NO_SR		RESET
#define I2C_SR			SET

// Base addresses of Flash and SRAM
#define FLASH_BASEADDR			0x08000000U //512Kb
#define SRAM_BASEADDR			0x20000000U //96kb
#define ROM_BASEADDR			0x1FFF0000U //30Kb
#define RCC_BASEADDR			0x40023800U
//AHBx and APBc Bus Peripheral base Addresses
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR 	0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
//Base Addresses of all peripheral which are hanging on AHB1 bus that we gonna use ofc
#define GPIOA_BASEADDR			AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR			0x40020400U
#define GPIOC_BASEADDR			0x40020800U
#define GPIOD_BASEADDR			0x40020c00U
#define GPIOE_BASEADDR			0x40021000U
#define GPIOH_BASEADDR			0x40021c00U
//Base Addresses of all peripheral which are hanging on APB1 bus that we gonna use ofc
#define I2C1_BASEADDR			0x40005400U
#define I2C2_BASEADDR			0x40005800U
#define I2C3_BASEADDR			0x40005C00U
#define SPI1_BASEADDR			0x40013000U
#define SPI2_BASEADDR			0x40003800U
#define SPI3_BASEADDR			0x40003C00U
#define SPI4_BASEADDR			0x40013400U
#define USART2_BASEADDR			0x40004400U
//Base Addresses of all peripheral which are hanging on APB2 bus that we gonna use ofc
#define SPI1_BASEADDR			0x40013000U
#define SPI4_BASEADDR			0x40013400U
#define USART1_BASEADDR			0x40011000U
#define USART6_BASEADDR			0x40011400U
#define EXTI_BASEADDR			0x40013C00U
#define SYSCFG_BASEADDR			0x40013800U
//NVIC Base Addresses
#define NVIC_ISER_BASEADDR		0xE000E100U
#define NVIC_ICER_BASEADDR		0XE000E180U
#define NVIC_IPR_BASEADDR		0xE000E400U

//Structure of GPIO peripheral
typedef struct
{
	__vo uint32_t MODER; // GPIO port mode register @ Offset = 0x00
	__vo uint32_t OTYPER; // GPIO port output type register 0x04
	__vo uint32_t OSPEEDR; // GPIO port output speed register  0x08
	__vo uint32_t PUPDR; // GPIO port pull-up/pull-down register 0x0C
	__vo uint32_t IDR; // GPIO port input data register 0x10
	__vo uint32_t ODR ; // GPIO port output data register  0x14
	__vo uint32_t BSRR; // GPIO port bit set/reset register 0x18
	__vo uint32_t LCKR; // GPIO port configuration lock register 0x1C
	__vo uint32_t AFR[2]; // GPIO alternate function low and high register AFR[0] = 0x20 / AFR[1] = 0x24

}GPIO_RegDef_t;
/*
 * Structure of SPI Peripheral
 */
typedef struct
{
	__vo uint32_t CR1; // SPI control register 1
	__vo uint32_t CR2; // SPI control register 2
	__vo uint32_t SR; // SPI status register
	__vo uint32_t DR; // SPI data register
	__vo uint32_t CRCPR; //SPI CRC polynomial register
	__vo uint32_t RXCRCR; //SPI RX CRC register
	__vo uint32_t TXCRCR; //SPI TX CRC register
	__vo uint32_t I2SCFGR; //SPI_I2S configuration register
	__vo uint32_t I2SPR; //SPI_I2S Prescaler register

}SPI_RegDef_t;
/*
 * Structure of I2C Peripheral
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;
/*
 * Structure of USART Peripheral
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USART_RegDef_t;


//Structure of RCC peripheral
typedef struct
{
	__vo uint32_t CR; //0x00
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
		 uint32_t reserved1; // reserved 0x48
		 uint32_t reserved2;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
		 uint32_t reserved3;
		 uint32_t reserved4;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
		 uint32_t reserved5;
		 uint32_t reserved6;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
		 uint32_t reserved7;
		 uint32_t reserved8;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
		 uint32_t reserved9;
		 uint32_t reserved10;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
		 uint32_t reserved11;
		 uint32_t reserved12;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	     uint32_t reserved13;
		 uint32_t reserved14;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	 	 uint32_t reserved15;
	__vo uint32_t DCKCFGR; //0x8C

}RCC_RegDef_t;
//Structure of EXTI peripheral
typedef struct
{
	__vo uint32_t IMR; // Interrupt mask register
	__vo uint32_t EMR; // Event mask register
	__vo uint32_t RTSR; // Rising trigger selection register
	__vo uint32_t FTSR; // Falling trigger selection register
	__vo uint32_t SWIER; // Software interrupt event register
	__vo uint32_t PR ; // Pending register

}EXTI_RegDef_t;
//Structure of SYSCFG peripheral
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
		 uint32_t RESERVED1;
		 uint16_t RESERVED2;
	__vo uint32_t CMPCR ;

}SYSCFG_RegDef_t;
//Structure of NVIC peripheral
typedef struct
{
	__vo uint32_t ISER[7];

}NVIC_ISER_RegDef_t;
typedef struct
{
	__vo uint32_t ICER[7];

}NVIC_ICER_RegDef_t;
typedef struct
{
	__vo uint32_t IPR[59];

}NVIC_IPR_RegDef_t;

/*
 * IRQ Number for stm32f401xx MCU
 */
#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_EXTI5_9		23
#define IRQ_EXTI10_15	40

#define IRQ_SPI1		35
#define IRQ_SPI2		36
#define IRQ_SPI3		51
#define IRQ_SPI4		84

#define IRQ_NO_I2C1_EV	31
#define	IRQ_NO_I2C_ER	32


// peripheral definitions ( peripheral base addresses typecasted to xxx_RegDef_t )

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define NVIC_ISER	((NVIC_ISER_RegDef_t*)NVIC_ISER_BASEADDR)
#define NVIC_ICER	((NVIC_ICER_RegDef_t*)NVIC_ICER_BASEADDR)
#define NVIC_IPR	((NVIC_IPR_RegDef_t*)NVIC_IPR_BASEADDR)
#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)
#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)
#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)


//>>>>>>>>>>>>>>GPIO Enable and Disable Clock
// Clock Enable Macros For "GPIOx" Peripherals
#define GPIOA_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()	(RCC -> AHB1ENR |= (1 << 7))
// Clock DISABLE Macros For "GPIOx" Peripherals
#define GPIOA_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()	(RCC -> AHB1ENR &= ~(1 << 7))
//////////////////////////////////

//>>>>>>>>>>>>>>>I2C Enable and Disable Clock
// Clock Enable Macros For "I2C" Peripherals
#define I2C1_PCLK_EN()	(RCC -> APB1ENR |=(1 << 21))
#define I2C2_PCLK_EN()	(RCC -> APB1ENR |=(1 << 22))
#define I2C3_PCLK_EN()	(RCC -> APB1ENR |=(1 << 23))
// Clock Disable Macros For "I2c" Peripherals
#define I2C1_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 23))
///////////////////////////////////

//>>>>>>>>>>>>>>>>>>SPI Enable and Disable Clock
// Clock Enable Macros For "SPI" Peripherals
#define SPI1_PCLK_EN()	(RCC -> APB2ENR |=(1 << 12))
#define SPI2_PCLK_EN()	(RCC -> APB1ENR |=(1 << 14))
#define SPI3_PCLK_EN()	(RCC -> APB1ENR |=(1 << 15))
#define SPI4_PCLK_EN()	(RCC -> APB2ENR |=(1 << 13))
// Clock Disable Macros For "SPI" Peripherals
#define SPI1_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 13))
///////////////////////////////////

//>>>>>>>>>>>>>>>>USART Enable and Disable Clock
// Clock Enable Macros For "USART" Peripherals
#define USART1_PCLK_EN()	(RCC -> APB2ENR |=(1 << 4))
#define USART2_PCLK_EN()	(RCC -> APB1ENR |=(1 << 17))
#define USART6_PCLK_EN()	(RCC -> APB2ENR |=(1 << 5))
// Clock Disable Macros For "USART" Peripherals
#define USART1_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC -> APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 5))
///////////////////////////////////////

//>>>>>>>>>>>>>>>>SYSCFG Enable and Disable Clock
// Clock Enable Macros For "SYSCFG" Peripherals
#define SYSCFG_PCLK_EN()	(RCC -> APB2ENR |=(1 << 14))
// Clock Disable Macros For "SYSCFG" Peripherals
#define SYSCFG_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 14))
////////////////////////////////
/*
 * macros to reset GPIOx peripheral
 */
#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET() do{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_REG_RESET() do{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOH_REG_RESET() do{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);}while(0)
/*
 * macros to reset SPIx peripheral
 */
#define SPI1_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB1RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET() do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET() do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET() do{RCC->APB2RSTR |= (1 << 13); RCC->APB1RSTR &= ~(1 << 13);}while(0)
/*
 * macros to reset I2Cx peripheral
 */
#define I2C1_REG_RESET() do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET() do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET() do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)
/*
 * macros to reset USARTx peripheral
 */
#define USART1_REG_RESET() do { (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); } while(0)
#define USART6_REG_RESET() do { (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); } while(0)

/****************************
 * Bit Position	definitions of SPI_CR1
 ****************************/
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_SPI_CR1_	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15
/****************************
 * Bit Position	definitions of SPI_CR2
 ****************************/
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
/****************************
 * Bit Position	definitions of SPI_SR
 ****************************/
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRC_ERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
/****************************
 * Bit Position	definitions of I2C_CR1
 ****************************/
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMB_TYPE		3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NO_STRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15
/****************************
 * Bit Position	definitions of I2C_CR2
 ****************************/
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12
/****************************
 * Bit Position	definitions of I2C_SR1
 ****************************/
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_TIMEOUT			14
/****************************
 * Bit Position	definitions of I2C_SR2
 ****************************/
#define I2C_SR2_MLS				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_DUALF			7
/****************************
 * Bit Position	definitions of I2C_CCR
 ****************************/
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15
/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9
/*
 * I2C Event Macros
 */
#define	I2C_EV_TX_CMPLT			0
#define	I2C_EV_RX_CMPLT			1
#define	I2C_EV_STOP				2
#define I2C_ER_ARLO				3
#define I2C_ER_AF				4
#define	I2C_ER_OVR				5
#define	I2C_ER_TIMEOUT			6
#define	I2C_ER_BERR				7


#include "stm32f401xx_gpio.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_usart_driver.h"
#include "stm32f401xx_rcc_driver.h"


#endif /* INC_STM32F401XX_H_ */
