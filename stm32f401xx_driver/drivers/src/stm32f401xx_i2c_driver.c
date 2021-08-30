/*
 * stm32f401xx_i2c_driver.c
 *
*
 * Title                 :   I2C Driver
 * Filename              :   stm32f401xx_i2c_driver.c
 * Author                :   Khldun Said
 * Origin Date           :   27/08/2021
 * Version               :   1.0.0
 * Compiler              :   GNU C/C++ for Arm® toolchain and GDB debugger
 * Target                :   NUCLEO-F401RE development board
 * Notes                 :   None
 */
/** @file TODO: MODULE.h
 *  @brief This module TODO: WHAT DO I DO?
 *
 *  This is the header file for the definition TODO: MORE ABOUT ME!
 */
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author         Description
*  27/08/2021   1.0.0   Khldun Said   Initial Release.
*/
/*
* Includes
*******************************************************************************/
#include <stdint.h>
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_rcc_driver.h"


/* Static Function prototypes
 * *******************************************
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteAdressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress);





static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);


}


static void I2C_ExecuteAdressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress <<1;
	SlaveAddress &= ~(1);
	pI2Cx->DR = SlaveAddress;

}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{

}










/* @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}

	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}

}

/* @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */



void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//Enalbe the clock for The i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);


	//ack control bit
	tempreg |= pI2CHandle->I2C_config.I2C_ACKControl <<10 ;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ filed of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle -> pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg =0;
	tempreg |= pI2CHandle->I2C_config.I2C_DeviceAddress <<1;
	tempreg |= (1 <<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CRR Calculations
	uint16_t ccr_value= 0 ;
	tempreg = 0;
	if(pI2CHandle->I2C_config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() /  (2* pI2CHandle->I2C_config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);

	}else {
		//mode is fast mode
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_config.I2C_FMDutyCycle <<14);
		if(pI2CHandle->I2C_config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() /  (3* pI2CHandle->I2C_config.I2C_SCLSpeed));
		}else {
			ccr_value = (RCC_GetPCLK1Value() /  (25* pI2CHandle->I2C_config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


}
/* @fn      		  - I2C_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		{
			I2C1_REG_RESET();
		}else if (pI2Cx == I2C2){
			I2C2_REG_RESET();
		}else if (pI2Cx == I2C3){
			I2C3_REG_RESET();
		}
}


/* @fn      		  - I2C_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,uint32_t len , uint8_t SlaveAddr,uint8_t Sr){

	//1. Generate the Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR
	// Note :until SB is cleared SCL Will stretched (pulled to low)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3.send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecutedAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that Address phase is completed by checking the ADDR flag  in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to low)
	I2c_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0

	while(len >0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		len--;

	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);













}


