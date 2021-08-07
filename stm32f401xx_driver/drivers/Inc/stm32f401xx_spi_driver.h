/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: May 22, 2021
 *      Author: kh
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_


#include"stm32f401xx.h"

/*
 * Configuration structure for SPIx peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t	*pSPIx;  /*this holds the address of SPIx(1,2,3,4) peripheral */
	SPI_Config_t    SPIConfig;
}SPI_Handle_t;


/*
 * @ SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1



/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1		//Full Duplex
#define SPI_BUS_CONFIG_HD				2		//Half Duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	34		//SIMPLEX receive only


/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1


/*
 * @SPI_SSM
 */

#define	SPI_SSM_DI							0
#define SPI_SSM_EN							1


/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG   (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG  (1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG  (1<<SPI_SR_BUSY)
/***********************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs check the function definitions
************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDe );

/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHnadling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control API's
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDe );

#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
