/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Aug 12, 2020
 *      Author: kh
 */


#include  "stm32f401xx_gpio_driver.h"



/***********************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
************************************************************************************/


 /* @fn      		  - GPIO_PeriClockControl
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

void GPIO_PerClockControl(GPIO_RegDef_t *pGPIOx,uint8_t Enorde)
{
	if(Enorde == ENABLE)
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
	}else{

		if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI	();
				}else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}

		}
}


/* @fn      		  - GPIO_Init
*
* @brief             - This function initate the GPIO PERPHIRALS Using GPIO Regsiter
* @param[in]         - GPIO_Handle_t structure
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle){
	uint32_t temp =0;

	//1. Configure the mode of GPIO pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//The uninterrupt mode
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle-> pGPIOX->MODER &= ~( 0x3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOX->MODER |=temp;
	}else{
		//this part will code later
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT){
			//1.Configure the FTSR
			EXTI->FTSR |=(1 <<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR
			EXTI->RTSR &=~(1 <<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT){
			//.1 Configure the RTSR
			EXTI->RTSR |=(1 <<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the FTSR
			EXTI->FTSR &=~(1 <<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT){
			//1.Congiure both FTSR and RTSR
			EXTI->RTSR |= (1 <<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 <<pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2.configure Sysconfig
		uint8_t temp1,temp2;
		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber /4;
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIO_Handle->pGPIOX);

		SYSCFG->SYSCFG_EXTICR[temp1] =portcode << ( temp2 * 4);

		//3.enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2.configure the speed

	temp =0;
	temp =(pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOX->OSPEEDR &= ~( 0x3 << ( 2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOX->OSPEEDR |=temp;

	//3. configure the pupd settings
	temp = 0;
	temp =(pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOX->PUPDR |=temp;

	//4. configure the optype

	temp=(pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOX->OTYPER |=temp ;

	//5. configure the alt functionality
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//
		uint8_t temp1,temp2;
		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIO_Handle->pGPIOX->AFR[temp1] &=(0xF << (4*temp2));
		pGPIO_Handle ->pGPIOX->AFR[temp1] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2));
	}

}
/* @fn      		  - GPIO_Deinit
*
* @brief             - This function resets peripheral
*
* @param[in]         - base address of the gpio peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void GPIO_Deinit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}






}

/* @fn      		  - GPIO_PeriClockControl
*
* @brief             - This function reads the value of input pin
*
* @param[in]         - base address of the gpio peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            - 0 or 1
*
* @Note              -  none

*/uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{

	uint8_t value ;
	value =(uint8_t) ((pGPIOx ->IDR >> PinNumber) & 0x00000001);
	return value;

}

/* @fn      		  - GPIO_PeriClockControl
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

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/* @fn      		  - GPIO_WriteToOutPin
*
* @brief             - This function writes value to the gpio pin (1 or 0 )
*
* @param[in]         - base address of the gpio peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
			//Write 1 to the output data register at the bit
		pGPIOx->ODR |= (1 << PinNumber);

	}else
	{
		pGPIOx->ODR &=~(1 << PinNumber);
	}

}



/* @fn      		  - GPIO_PeriClockControl
*
* @brief             - This function enables or disables peripheral clock for the given GPIO port
*
* @param[in]         - base address of the gpio peripheral
* @param[in]         - pin number
* @param[in]         - MACROS GPIO_PIN_SET OR GPIO_PIN_RESET
*
* @return            -  none
*
* @Note              -  none

*/


void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){

	pGPIOx->ODR = Value;

}

/* @fn      		  - GPIO_ToggleOutputPin
*
* @brief             - This function toggle output state form LOW to HIGH and
*
* @param[in]         - base address of the gpio peripheral
* @param[in]         - Pin Number
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber);

}

/* @fn      		  - GPIO_PeriClockControl
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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if (EnorDi == ENABLE){

		if(IRQNumber <= 31){
			//Program ISER0 Register
			*NVIC_ISER0 |= (1<< IRQNumber);

		}else if (IRQNumber >= 32 && IRQNumber <=63){

			//Program ISER1 Register
			*NVIC_ISER1 |= (1<< (IRQNumber%32));


		}else if (IRQNumber >= 64 && IRQNumber <=96){

			//Program ISER2 Register
			*NVIC_ISER2 |= (1<< (IRQNumber%64));

		}

	}else{
		if(IRQNumber <= 31){
					//Program I	cER0 Register
					*NVIC_ICER0 |= (1<< IRQNumber);

				}else if (IRQNumber >= 32 && IRQNumber <=63){

					//Program ICER1 Register
					*NVIC_ICER1 |= (1<< (IRQNumber%32));


				}else if (IRQNumber >= 64 && IRQNumber <=96){

					//Program ICER2 Register
					*NVIC_ICER2 |= (1<< (IRQNumber%64));
				}

	}

}

/* @fn      		  -  GPIO_IRQHnadling
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	//1.first lets find out the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber %4;

	uint8_t shift_amount = (8 * iprx_section) + (8 -NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR+ iprx) |= (IRQPriority << shift_amount);

}




/* @fn      		  -  GPIO_IRQHnadling
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

void GPIO_IRQHnadling(uint8_t PinNumber){

	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR &(1 << PinNumber)){
		//CLEAR
		EXTI->PR |= (1 << PinNumber);
	}

}

