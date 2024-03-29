/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 1, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include<stdint.h>
#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peipheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */

typedef struct {
	SPI_RegDef_t *pSPIx; /* This holds the base address of SPIx(x:0,1,2) peripheral */
	SPI_Config_t SPIConfig;
}SPI_Handle_t;



/* Peripheral Clock setup */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t clkStatus);



/* Init and Deinit */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);



/* Data send and receive */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


/* IRQ Configuration and ISR handling */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t clkStatus);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/* Other peripheral control APIs */

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
