/*
 * 003bush_botton.c
 *
 *  Created on: Aug 15, 2020
 *      Author: kh
 */


#include "stm32f401xx.h"


#define LOW 			0

void Delay(void){
	for(uint32_t i =0; i<500000;i++);
}


int main (void)
{
	GPIO_Handle_t Gpio_led,Gpio_btn;

	Gpio_led.pGPIOX = GPIOA;
	Gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpio_led.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_PP;
	Gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PerClockControl(GPIOA, ENABLE);

	GPIO_Init(&Gpio_led);

	Gpio_btn.pGPIOX =GPIOC;
	Gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	Gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PerClockControl(GPIOC, ENABLE);

	GPIO_Init(&Gpio_btn);

	while (1)
	{
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13)== LOW)
		{
			Delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);


		}


	}









}
