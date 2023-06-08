/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 1, 2023
 *      Author: user
 */

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"



/* Peripheral Clock setup */

/*****************************************************************************************
 * @func               - SPI_PeriClockControl
 *
 * @brief              - This function enables or disables peripheral clock for the given
 	 	 	 	 	 	 GPIO port
 *
 * @param[in]          - base address of the GPIO peripheral
 * @param[in]          - Enable or Disable macros
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t clkStatus) {

	if(clkStatus == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else {
			// Clock disable - do later
		}
	}

}



/* Init and Deinit
 *
 * */

/*****************************************************************************************
 * @func               - SPI_Init
 *
 * @brief              -
 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void SPI_Init(SPI_Handle_t *pSPIHandle) {

}



/*****************************************************************************************
 * @func               - SPI_DeInit
 *
 * @brief              -

 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

}





/* Data send and receive
 *
 *  */


/*****************************************************************************************
 * @func               - SPI_SendData
 *
 * @brief              -

 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

}



/*****************************************************************************************
 * @func               - SPI_ReceiveData
 *
 * @brief              -

 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {


}



/* IRQ Configuration and ISR handling
 *
 * */



/*****************************************************************************************
 * @func               -  SPI_IRQInterruptConfig
 *
 * @brief              -
 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t clkStatus){


}




/*****************************************************************************************
 * @func               - GPIO_IRQPriorityConfig
 *
 * @brief              -

 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {


}




/*****************************************************************************************
 * @func               - GPIO_IRQHandling
 *
 * @brief              -

 *
 * @param[in]          -
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */
void SPI_IRQHandling(SPI_Handle_t *pHandle) {


}


/* Other peripheral control APIs */


