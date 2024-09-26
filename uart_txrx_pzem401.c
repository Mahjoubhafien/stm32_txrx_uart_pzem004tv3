/*
 * stm32_txrx_uart_pzem004tv3.c
 *
 *  Created on: Sep 19, 2024
 *      Author: Mahjoubhafien
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f401xx.h"

//usart1 IRQ = 37
USART_Handle_t USART_Handle;

void delay()
{
	for(uint32_t i=0 ; i < 500000/2 ; i ++);
}
void delay1()
{
    // Total cycles for 364 µs at 84 MHz
    for (uint32_t i = 0; i < 30607; i++);
}
void Button_init(void)
{
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControle = GPIO_NO_PUPD;
	GPIO_PClkControl(GPIOC, ENABLE);
	GPIO_Init(&GpioButton);
}

void USART1_GPIO_Init(void)
{
	GPIO_Handle_t USART_gpio_setup;
	USART_gpio_setup.pGPIOx = GPIOB;
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinOPType =  GPIO_OP_TYPE_PP;
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinPuPdControle = GPIO_PIN_PU ;
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinAltFunMode = 7 ;

	GPIO_PClkControl(GPIOB, ENABLE);

	//TX
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinNumber = 7 ;
	GPIO_Init(&USART_gpio_setup);
	//RX
	USART_gpio_setup.GPIO_PinConfig.GPIO_PinNumber = 6 ;
	GPIO_Init(&USART_gpio_setup);
}

void USART1_Init(void)
{
	USART_Handle.pUSARTx = USART1;
	USART_Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART_Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;

	USART_Init(&USART_Handle);

}
uint8_t DATA[8] = {0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A,0x64 , 0x64} ;
//uint8_t DATA[] = {0x01, 0x03} ;
uint8_t len = sizeof(DATA);
uint8_t rxCmplt = RESET;

int main(void)
{
	uint8_t rxbuffer[1024];

	printf("START\n ");

	Button_init();

	//This function is used to initialize the GPIO Pins to behave as USART2 pins
	USART1_GPIO_Init();


	//This function is used to initialize the USART1 peripherla parameters
	USART1_Init();

	//Enable USART1
	USART_PeripheralControl(USART1, ENABLE);

	//Enable interrupt from usart1
	USART_IRQConfiq(37, 1, ENABLE);


	while (1) {

		//wait till button is pressed
		while (GPIO_ReadPin(GPIOC, 13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//Receive data interrupt based API
		while (USART_ReceiveDataIT(&USART_Handle, rxbuffer, 25)!= USART_READY);

		//Send Data blocking mode
		USART_SendData(&USART_Handle, DATA, len);

        printf("Transmitted: ");
        for (int i = 0; i < len; i++) {
            printf("%02X ", DATA[i]);
        }
        printf("\n");

    	//Now lets wait until all the bytes are received from the arduino .
    	//When all the bytes are received rxCmplt will be SET in application callback
    	while(rxCmplt != SET);

        printf("Received: ");
        for (int i = 0; i < 25; i++) {
            printf("%02X ", rxbuffer[i]);
        }
        printf("\n");

    	//invalidate the flag
    	rxCmplt = RESET;


}
	return 0;

}
void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USART_Handle);
}

void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
