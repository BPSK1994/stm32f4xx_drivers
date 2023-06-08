/*
 * 001LedToggle.c
 *
 *  Created on: May 24, 2023
 *      Author: user
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void) {
	for(uint32_t i = 0; i < 300000; i++);
}


int main(void) {

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	while(1) {
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}



