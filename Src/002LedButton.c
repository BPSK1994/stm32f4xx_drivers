/*
 * 002LedButton.c
 *
 *  Created on: May 29, 2023
 *      Author: user
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void) {
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void) {

	GPIO_Handle_t GpioLed, GpioBtn;

	// LED
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	// Button
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IP;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioBtn);

	while(1) {

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}

	}

	return 0;
}

