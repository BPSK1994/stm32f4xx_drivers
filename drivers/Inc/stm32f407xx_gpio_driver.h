/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 23, 2023
 *      Author: user
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"




/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;                 /* Possible values from @GPIO PIN MODES */
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct {

	GPIO_RegDef_t *pGPIOx;            // Pointer to hold the base address of the GPIO Port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;  // This holds GPIO pin configuration settings

}GPIO_Handle_t;


/* @GPIO PIN MODES */
/* GPIO pins possible mode macros */
#define GPIO_MODE_IP       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFUNC  2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_RFT   6


/* GPIO pins possible output type macros */
#define GPIO_OUT_PUSH_PULL  0
#define GPIO_OUT_OPEN_DRAIN 1


/* GPIO pins possible output speed macros */
#define GPIO_LOW_SPEED        0
#define GPIO_MED_SPEED        1
#define GPIO_HIGH_SPEED       2
#define GPIO_VERY_HIGH_SPEED  3


/* GPIO pins pull-up/pull-down configuration macros */
#define GPIO_NO_PUPD      0
#define GPIO_PU           1
#define GPIO_PD           2

/* GPIO pin numbers */
#define GPIO_PIN_NO_0            0
#define GPIO_PIN_NO_1            1
#define GPIO_PIN_NO_2            2
#define GPIO_PIN_NO_3            3
#define GPIO_PIN_NO_4            4
#define GPIO_PIN_NO_5            5
#define GPIO_PIN_NO_6            6
#define GPIO_PIN_NO_7            7
#define GPIO_PIN_NO_8            8
#define GPIO_PIN_NO_9            9
#define GPIO_PIN_NO_10           10
#define GPIO_PIN_NO_11           11
#define GPIO_PIN_NO_12           12
#define GPIO_PIN_NO_13           13
#define GPIO_PIN_NO_14           14
#define GPIO_PIN_NO_15           15



/* Peripheral Clock setup */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t clkStatus);


/* GPIO Init and DeInit*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/* GPIO Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);


/* GPIO Interrupt Control */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t clkStatus);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
