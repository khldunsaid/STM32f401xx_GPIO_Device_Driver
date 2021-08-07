/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: May 22, 2021
 *      Author: kh
 */

#include <stdint.h>
#include "stm32f401xx_spi_driver.h"


/*********************************************************************************/
 /* @fn      		  - SPI_PeriClockControl(
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI
 * @param[in]         - base address of the SPIx peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi )
{
	if(EnorDi == ENABLE){
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2){
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3){
				SPI3_PCLK_EN();
			}else if (pSPIx == SPI4){
				SPI4_PCLK_EN();
			}
		}
		else {
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if (pSPIx == SPI2){
				SPI2_PCLK_DI();
			}else if (pSPIx == SPI3){
				SPI3_PCLK_DI();
			}else if (pSPIx == SPI4){
				SPI4_PCLK_DI();
			}
		}

}

/*********************************************************************************/

/* @fn      		  - SPI_Init
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void SPI_Init(SPI_Handle_t *pSPI_Handle)
{

	//first lets configre the SPI_CR1 register
	uint32_t  tempreg =0 ;

	// Enable Colok Prephiral
	SPI_PeriClockControl(pSPI_Handle->pSPIx,ENABLE);
	//1. configure the device mode
	tempreg |= pSPI_Handle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//2. configure the bus config
	if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{

		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg &= ~(1<<SPI_CR1_RXONLY);
	}
	//3. configure the spi serial clock speed (baud rate)
	tempreg |= pSPI_Handle->SPIConfig.SPI_SclkSpeed <<SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPI_Handle->SPIConfig.SPI_DFF <<SPI_CR1_DFF	;

	//5. configure the CPOL
	tempreg |= pSPI_Handle->SPIConfig.SPI_CPOL <<SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPI_Handle->SPIConfig.SPI_CPHA <<SPI_CR1_CPHA;

	pSPI_Handle->pSPIx->CR1 |= tempreg;
}

/*********************************************************************************/
/* @fn      		  -  SPI_DeInit
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
				{
					SPI1_REG_RESET();
				}else if (pSPIx == SPI2){
					SPI2_REG_RESET();
				}else if (pSPIx == SPI3){
					SPI3_REG_RESET();
				}else if (pSPIx == SPI4){
					SPI4_REG_RESET();
				}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************************/

/* @fn      		  - SPI_SendData
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  This is Blocking Call

*/

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2.check the DFF bit in CR1
		if((pSPIx->CR1 & ( 1<< SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data in the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len --;
			Len --;
			(uint16_t*) pTxBuffer++;

		}else
		{
			//8 bit DFF
			//1. load the data in the DR
			pSPIx->DR = *(pTxBuffer);
			Len --;
			pTxBuffer++;

		}
	}

}



/*********************************************************************************/

/* @fn      		  - SPI_ReceiveData
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{

}


/*********************************************************************************/

/* @fn      		  -  SPI_IRQInterruptConfig
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*********************************************************************************/

/* @fn      		  - SPI_IRQPriorityConfig
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

/*********************************************************************************/
/* @fn      		  - SPI_IRQHnadling
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/

void SPI_IRQHnadling(SPI_Handle_t *pHandle)
{

}


/*********************************************************************************/
/* @fn      		  - SPI_IRQHnadling
*
* @brief             - This function enables or disables peripheral clock for the given SPI
* @param[in]         - base address of the SPIx peripheral
* @param[in]         - ENABLE or DISABLE macros
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDe )
{
	if(EnorDe == ENABLE)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SPE);
	}
}



