/*
 * stm32f401xx_i2c_driver.h
 *
 *
 * Title                 :   I2C Driver
 * Filename              :   stm32f401xx_i2c_driver.h
 * Author                :   Khldun Said
 * Origin Date           :   27/08/2021
 * Version               :   1.0.0
 * Compiler              :   GNU C/C++ for Arm® toolchain and GDB debugger
 * Target                :   NUCLEO-F401RE development board
 * Notes                 :   None
 */

/*****************************************************************************/
/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    	Version 	  Author         Description
*  27/08/2021   1.0.0   	Khldun Said  	Interface Created.
*
*****************************************************************************/
/** @file TODO: MODULE.h
 *  @brief This module TODO: WHAT DO I DO?
 *
 *  This is the header file for the definition TODO: MORE ABOUT ME!
 */

#ifndef INC_STM32F401XX_I2C_DRIVER_H_
#define INC_STM32F401XX_I2C_DRIVER_H_


#include"stm32f401xx.h"
/*
 * Configuration structure for I2Cx peripheral
 */


typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;

}I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_config;

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM  100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_SM2K 200000



/*
 * @I2C_DeviceAddress
 */

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE    	1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


/*
 * I2c Related status flags definitions
 */

#define I2C_FLAG_TXE 		(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXEN)
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR 		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)


#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET




/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* INC_STM32F401XX_I2C_DRIVER_H_ */



