/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 23, 2023
 *      Author: user
 */


#include "stm32f407xx_gpio_driver.h"

/* Peripheral Clock setup
 *
 */

/*****************************************************************************************
 * @func               - GPIO_PeriClockControl
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

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t clkStatus) {
	if(clkStatus == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		} else {
			// DO LATER
		}

	}
}


/*****************************************************************************************
 * @func               - GPIO_Init
 *
 * @brief              - This function enables or disables peripheral clock for the given
 	 	 	 	 	 	 GPIO port
 *
 * @param[in]          - base address of the GPIO peripheral
 *
 *
 * @return             - none
 *
 * @note               - none

 */


void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	// 1. Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // Setting

	} else {
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. Configure the Falling trigger selection register (FTSR)
			EXTI->FTSR = EXTI->FTSR | (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR = EXTI->RTSR & ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			// 1. Configure the Rising trigger selection register
			EXTI->RTSR = EXTI->RTSR | (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->FTSR = EXTI->FTSR & ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. Configure the Rising trigger register and Falling trigger register
			EXTI->RTSR = EXTI->RTSR | (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR = EXTI->FTSR | (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}


		// 2. Configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2 * 4);

		// 3. Enable the EXTI interrupt delivery IMR (Interrupt Mask Register)
		EXTI->IMR = EXTI->IMR | (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}

	temp = 0;

	// 2. Configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // Setting

	temp = 0;


    // 3. Configure the Pull-up/pull-down settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; // Setting

	temp = 0;


	// 4. Configure the output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; // Setting

	temp = 0;


	//5. Configure the alternate functionality


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUNC) {
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2)); // Setting

	}


}


/*****************************************************************************************
 * @func               - GPIO_DeInit
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}


/*****************************************************************************************
 * @func               - GPIO_ReadFromInputPin
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);

	return value;
}


/*****************************************************************************************
 * @func               - GPIO_ReadFromInputPort
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR);

	return value;
}


/*****************************************************************************************
 * @func               - GPIO_WriteToOutputPin
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             -
 *
 * @note               -

 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {

	if(value == GPIO_PIN_SET) {
			// Write 1 to the ODR at the bit field corresponding to the pin
			pGPIOx->ODR |= (1 << pinNumber);
		} else {
			// Write 0
			pGPIOx->ODR &= ~(1 << pinNumber);
		}
}


/*****************************************************************************************
 * @func               - GPIO_WriteToOutputPort
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

	pGPIOx->ODR = value;

}



/*****************************************************************************************
 * @func               - GPIO_ToggleOutputPin
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {

	pGPIOx->ODR = pGPIOx->ODR ^ (1 << pinNumber);

}



/*****************************************************************************************
 * @func               - GPIO_IRQConfig
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t clkStatus) {

	if(clkStatus == ENABLE) {

		if(IRQNumber <= 31) {

			// Program ISER0 register
			*(NVIC_ISER0) = *(NVIC_ISER0) | (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64) { // 32 to 63

			// Program ISER1 register
			*(NVIC_ISER1) = *(NVIC_ISER1) | (1 << (IRQNumber % 32));

		} else if(IRQNumber >= 64 && IRQNumber < 96) {

			// Program ISER2 register
			*(NVIC_ISER2) = *(NVIC_ISER2) | (1 << (IRQNumber % 32));

		}
	} else {
		if(IRQNumber <= 31) {

			// Program ICER0 register
			*(NVIC_ICER0) = *(NVIC_ICER0) | (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64) {

			// Program ICER1 register
			*(NVIC_ICER1) = *(NVIC_ICER1) | (1 << (IRQNumber % 32));

		} else if(IRQNumber >= 64 && IRQNumber < 96) {

			// Program ICER2 register
			*(NVIC_ICER2) = *(NVIC_ICER2) | (1 << (IRQNumber % 32));

		}
	}

}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

	// 1. Find out the Interrupt Priority Register (IPR)

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_OF_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + (iprx * 4)) = *(NVIC_PR_BASEADDR + (iprx * 4)) | (IRQPriority << shift_amount );
}



/*****************************************************************************************
 * @func               - GPIO_IRQHandling
 *
 * @brief              -
 *
 * @param[in]          -
 *
 *
 * @return             - none
 *
 * @note               - none

 */

void GPIO_IRQHandling(uint8_t pinNumber) {

	// 1. Clear the EXTI PC register corresponding to the pin number

	if(EXTI->PR & (1 << pinNumber)) {
		// Clear
		EXTI->PR = EXTI->PR | (1 << pinNumber);
	}
}
