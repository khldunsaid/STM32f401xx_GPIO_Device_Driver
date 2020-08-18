/*
 * 002led_toggle_OD.c
 *
 *  Created on: Aug 15, 2020
 *      Author: kh
 */


#include "stm32f401xx.h"

void Delay(void){
	for(uint32_t i =0; i<500000;i++);
}


int main (void)
{
	GPIO_Handle_t Gpio_led;

	Gpio_led.pGPIOX = GPIOA;
	Gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpio_led.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_OD;
	Gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PerClockControl(GPIOA, ENABLE);

	GPIO_Init(&Gpio_led);

	while (1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		Delay();


	}









}
