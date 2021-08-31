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
*  31/08/2021	1.0.0	Khldun Said   Add Master_Fecive_Function.
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
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress);
static void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx);





static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);


}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress <<1;
	SlaveAddress &= ~(1);
	pI2Cx->DR = SlaveAddress;

}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress <<1;
	SlaveAddress |= 1;
	pI2Cx->DR = SlaveAddress;

}


static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{

	uint32_t dummy_read;

			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;


}


static void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);


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
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl <<10 ;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Configure the FREQ filed of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle -> pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg =0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
	tempreg |= (1 <<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CRR Calculations
	uint16_t ccr_value= 0 ;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() /  (2* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);

	}else {
		//mode is fast mode
		tempreg |= (1<<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value() /  (3* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else {
			ccr_value = (RCC_GetPCLK1Value() /  (25* pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is standadrd mode
		tempreg = (RCC_GetPCLK1Value()/1000000U)+1;
	}else
	{
		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) /1000000000U);
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);


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


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx-> SR1 & FlagName){
		return FLAG_SET;
	}
  return FLAG_RESET;
}


/* @fn      		  - I2C_MasterSendData
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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,uint32_t len , uint8_t SlaveAddr){

	//1. Generate the Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
		//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
		while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3.send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that Address phase is completed by checking the ADDR flag  in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle);

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
	I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//generate STOP condition
		I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));



		//read data in to buffer
		*pRxBuffer =pI2CHandle->pI2Cx->DR;


	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);


			}

			//read the data from data register in to buffer
			*pRxBuffer =pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);

}



void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == I2C_ACK_ENABLE){
		//Enable ACK (10th bit of CR1 register)
		pI2Cx->CR1 |= (1 <<I2C_CR1_ACK) ;

	}else{
		//Disable ACK (10th bit of CR1 register)
		pI2Cx->CR1 &= ~(1 <<I2C_CR1_ACK) ;

	}
}




