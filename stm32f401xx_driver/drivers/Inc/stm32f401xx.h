/*
 * stm32f401xx.h
 *
 *  Created on: Aug 11, 2020
 *      Author: kh
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

#define __vo   volatile



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Adrresses
 */
#define NVIC_ISER0           				((__vo uint32_t*)0x0E000E100)
#define NVIC_ISER1           				((__vo uint32_t*)0x0E000E104)
#define NVIC_ISER2           				((__vo uint32_t*)0x0E000E108)
#define NVIC_ISER3           				((__vo uint32_t*)0x0E000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Adrresses
 */
#define NVIC_ICER0           				((__vo uint32_t*)0x0E000E180)
#define NVIC_ICER1           				((__vo uint32_t*)0x0E000E184)
#define NVIC_ICER2           				((__vo uint32_t*)0x0E000E188)
#define NVIC_ICER3          				((__vo uint32_t*)0x0E000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 					((__vo uint32_t*)0xE000E4000)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


/*
 *  Base Addresses of Flash and SRAM Memories
 */

#define FLASH_BASEADDR						0x80000000U	 							 //Base Address of 512KB flash memory
#define SRAM_BASEADDR						0x20000000U								 //Base Address of 96KB SRAM memory
#define ROM_BASEADDR						0x1FFF0000U								 //Base Address of 30KB System memory (ROM)

/*
 * AHBx AND APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR						0x40000000		   						//Base Address of peripherals on memory map
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR  					 	//Base Address of APB1 Bus (Advanced Peripheral Bus)
#define APB2PERIPH_BASEADDR					0x40010000								//Base Address of APB2 Bus (Advanced Peripheral Bus)
#define AHB1PERIPH_BASEADDR					0x40020000								//Base Address of AHB1 Bus (Advanced High-performance Bus)
#define AHB2PERIPH_BASEADDR					0x50000000								//Base Address of AHB2 Bus (Advanced High-performance Bus)


/*
 * Base Addresses of peripherals which are hanging on APB1 bus
 */
#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR+0x0000)			//General-purpose timer2
#define TIM3_BASEADDR						(APB1PERIPH_BASEADDR+0x0400)			//General-purpose timer3
#define TIM4_BASEADDR						(APB1PERIPH_BASEADDR+0x0800)			//General-purpose timer4
#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR+0x0C00)			//General-purpose timer5
#define RTC_BKP_REGISTERS_BASEADDR 			(APB1PERIPH_BASEADDR+0x2800)			//Real-time clock (RTC)
#define WWDG_BASEADDR 						(APB1PERIPH_BASEADDR+0x2C00)			//Window watchdog (WWDG)
#define IWDG_BASEADDR 						(APB1PERIPH_BASEADDR+0x3000)			//Independent watchdog
#define I2S2EXT_BASEADDR					(APB1PERIPH_BASEADDR+0x3400)
#define SPI2_I2S2_BASEADDR					(APB1PERIPH_BASEADDR+0x3800)
#define SPI3_I2S3_BASEADDR					(APB1PERIPH_BASEADDR+0x3C00)
#define I2S3EXT_BASEADDR					(APB1PERIPH_BASEADDR+0x4000)
#define USART2_BASEADDR 					(APB1PERIPH_BASEADDR+0x4400)
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR+0x5C00)
#define PWR_BASEADDR 						(APB1PERIPH_BASEADDR_0x7000)

/*
 *Base Addresses of peripherals which are hanging on APB2 bus
 */


#define TIM1_BASEADDR						(APB2PERIPH_BASEADDR+0x0000)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR+0x1400)
#define ADC1_BASEADDR						(APB2PERIPH_BASEADDR+0x2C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR+0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR+0x3400)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR+0x3800)
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR+0x3C00)
#define TIM9_BASEADDR	`					(APB2PERIPH_BASEADDR+0x4000)
#define TIM10_BASEADDR 						(APB2PERIPH_BASEADDR+0x4400)
#define TIM11_BASEADDR				      	(APB2PERIPH_BASEADDR+0x4800)
/*
 * Base Addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR+0x1C00)
#define CRC_BASEADDR 						(AHB1PERIPH_BASEADDR+0x3000)
#define RCC_BASEADDR 						(AHB1PERIPH_BASEADDR+0x3800)
#define FLASH_INTERFACE_REGISTER_BASEADDR	(AHB1PERIPH_BASEADDR+0x3C00)
#define DMA1_BASEADDR						(AHB1PERIPH_BASEADDR+0x6000)
#define DMA2_BASEADDR 						(AHB1PERIPH_BASEADDR+0x6400)

/*
 * Base Addresses of peripherals which are hanging on AHB2 bus
 */
#define USB_OTG_FS_BASEADDR					(AHB2_BASEADDR+0x0000)



/**********************************peripheral register definition structures **********************************/

typedef struct
{
	__vo uint32_t MODER;			//GPIO port mode register																		Address offset 0x00
	__vo uint32_t OTYPER;			//GPIO port output type register																Address offset 0x04
	__vo uint32_t OSPEEDR;			//GPIO port output speed register																Address offset 0x08
	__vo uint32_t PUPDR;			//GPIO port pull-up/pull-down register															Address offset 0x0C
	__vo uint32_t IDR;				//GPIO port input data register																	Address offset 0x10
	__vo uint32_t ODR;				//GPIO port output data register																Address offset 0x14
	__vo uint32_t BSRR;				//GPIO port bit set/reset register																Address offset 0x18
	__vo uint32_t LCKR;				//GPIO port configuration lock register															Address offset 0x1C
	__vo uint32_t AFR[2];			//AFR[0] GPIO alternate function low register ,AFR[1] GPIO alternate function high register     Address offset 0x20 and 0x24
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;				//RCC clock control register																	Address offset: 0x00
	__vo uint32_t PLLCFGR;			//RCC PLL configuration register 																Address offset: 0x04
	__vo uint32_t CFGR;				//RCC clock configuration register																Address offset: 0x08
	__vo uint32_t CIR;				//RCC clock interrupt register																	Address offset: 0x0C
	__vo uint32_t AHB1RSTR;			//RCC AHB1 peripheral reset register															Address offset: 0x10
	__vo uint32_t AHB2RSTR;			//RCC AHB2 peripheral reset register															Address offset: 0x14
	uint32_t Reserved[2];			//
	__vo uint32_t APB1RSTR;			//RCC APB1 peripheral reset register															Address offset: 0x20
	__vo uint32_t APB2RSTR;			//RCC APB2 peripheral reset register															Address offset: 0x24
	uint32_t Reserved1[2];
	__vo uint32_t AHB1ENR;			//RCC AHB1 peripheral clock enable register														Address offset: 0x30
	__vo uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable registe														Address offset: 0x34
	uint32_t Reserved2[2];
	__vo uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register 													Address offset: 0x40
	__vo uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register														Address offset: 0x44
	uint32_t Reserved3[2];
	__vo uint32_t AHB1LPENR;		//RCC AHB1 peripheral clock enable in low power mode register									Address offset: 0x50
	__vo uint32_t AHB2LPENR;		//RCC AHB2 peripheral clock enable in low power mode register									Address offset: 0x54
	uint32_t Reserved4[2];
	__vo uint32_t APB1LPENR;		//RCC APB1 peripheral clock enable in low power mode register									Address offset: 0x60
	__vo uint32_t APB2LPENR;		//RCC APB2 peripheral clock enabled in low power mode register									Address offset: 0x64
	uint32_t Reserved5[2];
	__vo uint32_t BDCR;				//RCC Backup domain control register 															Address offset: 0x70
	__vo uint32_t CSR;				//RCC clock control & status register															Address offset: 0x74
	uint32_t Reserved6[2];
	__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register 												Address offset: 0x80
	__vo uint32_t PLLI2SCFGRRe;		//RCC PLLI2S configuration register																Address offset: 0x84
	__vo uint32_t DCKCFGR;			//RCC Dedicated Clocks Configuration Register													Address offset: 0x8C


}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;					// Interrupt mask register															Address offset: 0x00
	__vo uint32_t EMR;					//Event mask register (EXTI_EMR)													Address offset: 0x04
	__vo uint32_t RTSR;					//Rising trigger selection register (EXTI_RTSR)										Address offset: 0x08
	__vo uint32_t FTSR;					//Falling trigger selection register (EXTI_FTSR)									Address offset: 0x0C
	__vo uint32_t SWIER;				//Software interrupt event register													Address offset: 0x10
	__vo uint32_t PR;					//Pending register 																	Address offset: 0x14

}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t SYSCFG_MEMRMP;		//SYSCFG memory remap register 													Address offset: 0x00
	__vo uint32_t SYSCFG_PMC;			//SYSCFG peripheral mode configuration register (SYSCFG_PMC)					Address offset: 0x04
	__vo uint32_t SYSCFG_EXTICR[4];		//SYSCFG external interrupt configuration register 								Address offset: 0x08-0x14
	uint32_t reseverd[2];				//																				Address offset: 0x18-0x1C
	__vo uint32_t SYSCFG_CMPCR;			//Compensation cell control register											Address offset: 0x20

}SYSCFG_RegDef_t;


/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |=(1<<4))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |=(1<<7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */





/*
 * Clock Enable Macros for SPIx peripheralsbu
 */



/*
 * Clock Enable Macros for USARTx peripherals
 */


/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<7))


/*
 * Clock Disable Macros for SPIx peripherals
 */


/*
 * Clock Disable Macros for USARTx peripherals
 */



/*
 * Clock Disable Macros for SYSCFG peripheral
 */




/*
 * Macro to reset GPIOx Preiphirals
 */
#define GPIOA_REG_RESET()					do{ (RCC->AHB1RSTR |=(1<<0));	(RCC->AHB1RSTR &=~(1<<0));} while(0)
#define GPIOB_REG_RESET()					do{ (RCC->AHB1RSTR |=(1<<1));	(RCC->AHB1RSTR &=~(1<<1));} while(0)
#define GPIOC_REG_RESET()					do{ (RCC->AHB1RSTR |=(1<<2));	(RCC->AHB1RSTR &=~(1<<2));} while(0)
#define GPIOD_REG_RESET()					do{ (RCC->AHB1RSTR |=(1<<3));	(RCC->AHB1RSTR &=~(1<<3));} while(0)

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:0)



/*
 * IRQ(Interrupt Request) Numbers of STM32F401RE MCU
 */
#define IRQ_NO_EXTI0  		  6
#define IRQ_NO_EXTI1  		  7
#define IRQ_NO_EXTI2 	 	  8
#define IRQ_NO_EXTI3  	 	  9
#define IRQ_NO_EXTI4  		  10
#define IRQ_NO_EXTI9_5    	  23
#define IRQ_NO_EXTI15_10      40


/*
 *  IRQ PRIORITY LEVEL
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15



/*
 * some Generic Macro
 */
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET


#include "stm32f401xx_gpio_driver.h"

#endif /* INC_STM32F401XX_H_ */
