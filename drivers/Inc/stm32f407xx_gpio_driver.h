/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jun 29, 2024
 *      Author: Jose Manuel Barajas Ramirez (barajas.jose.3d@gmail.com)
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* Handle structure of GPIO pin */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
	// Pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // This holds the pin configuration settings
} GPIO_Handle_t;

// Clock control API for GPIOs
void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// GPIO Init and DeInit APIs
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// GPIO read and write to port or pin
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// GPIO IRQ configuration and handling
void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQ_ProrityConfig(uint8_t IRQ_Number, uint32_t Interrupt_Prority);

// GPIO pin states
#define GPIO_PIN_SET 	 	1
#define GPIO_PIN_RESET 		0

// GPIO pin numbers
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

// GPIO pin modes
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOGE	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

// GPIO output types
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

// GPIO pin speeds
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

// GPIO pin pull-up/pull-down
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

// GPIO alternate function modes
#define GPIO_MODE_ALTFN_0			0
#define GPIO_MODE_ALTFN_1			1
#define GPIO_MODE_ALTFN_2			2
#define GPIO_MODE_ALTFN_3			3
#define GPIO_MODE_ALTFN_4			4
#define GPIO_MODE_ALTFN_5			5
#define GPIO_MODE_ALTFN_6     		6
#define GPIO_MODE_ALTFN_7			7
#define GPIO_MODE_ALTFN_8			8
#define GPIO_MODE_ALTFN_9			9
#define GPIO_MODE_ALTFN_10			10
#define GPIO_MODE_ALTFN_11			11
#define GPIO_MODE_ALTFN_12			12
#define GPIO_MODE_ALTFN_13			13
#define GPIO_MODE_ALTFN_14			14
#define GPIO_MODE_ALTFN_15			15

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
