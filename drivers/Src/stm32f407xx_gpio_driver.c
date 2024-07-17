/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 29, 2024
 *      Author: Jose Manuel Barajas Ramirez (barajas.jose.3d@gmail.com)
 */

#include "stm32f407xx_gpio_driver.h"
#include <stdio.h>

// Clock control API for GPIOs
void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi) {
	if(EnorDi == ENABLE){
		if (pGPIOx == GPIOA) {
			GPIOA_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOB) {
			GPIOB_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOF){
			GPIOF_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOG){
			GPIOG_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOI){
			GPIOI_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOJ){
			GPIOJ_CLOCK_ENABLE();
		}else if (pGPIOx == GPIOK){
			GPIOK_CLOCK_ENABLE();
		}
	}else{
		if (pGPIOx == GPIOA) {
			GPIOA_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOB) {
			GPIOB_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOC){
			GPIOC_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOD){
			GPIOD_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOE){
			GPIOE_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOF){
			GPIOF_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOG){
			GPIOG_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOH){
			GPIOH_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOI){
			GPIOI_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOJ){
			GPIOJ_CLOCK_DISABLE();
		}else if (pGPIOx == GPIOK){
			GPIOK_CLOCK_DISABLE();
		}
	}
}

// GPIO Init and Deinit APIs
void GPIO_Init(GPIO_Handle_t* pGPIOHandle) {
    uint32_t temp = 0;

    // Enable the peripheral clock
    GPIO_PeripClockControl(pGPIOHandle->pGPIOx, ENABLE);

    // Configure the modes of a GPIO pin
    // Configuring non interrupt modes
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOGE) {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
        pGPIOHandle->pGPIOx->MODER |= temp;
    } else {
        // It is interrupt mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
            // Configure the FTSR register
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the RTSR bit
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
            // Configure the RTSR register
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the FTSR bit
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
            // Configure the RTSR register
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Configure the FTSR register
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // configure the GPIO port selection in syscfg_EXTIcr
        // SYSCFG SYSCFG_EXTICR[x] configures which pin of which port issues interrupt
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = (GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx));
        // Enabling the clock for SYSCFG before configuring the registers for SYSCFG
        SYSCFG_CLOCK_ENABLE();
        SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

        // Enable the interrupt delivery in EXTI
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    temp = 0;

    // Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;

    // Configure the pupd control
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    // Configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    // Configure the alternate functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        // Configure the alternate function registers
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}else if (pGPIOx == GPIOJ){
		GPIOJ_REG_RESET();
	}else if (pGPIOx == GPIOK){
		GPIOK_REG_RESET();
	}
}

// GPIO read and write to port or pin
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber) {
	uint8_t value ;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001 );
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR );
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		// write 1 to the bit field corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// write 0 to the bit field corresponding pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint16_t Value) {
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber) {
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}

// GPIO IRQ configuration and handling
void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQ_Number <= 31) {
			// Configure ISER 0 register
			*NVIC_ISER0 |= ( 1 << IRQ_Number);
		} else if (IRQ_Number > 31 && IRQ_Number < 64){
			// Configure ISER 1 register
			*NVIC_ISER1 |= ( 1 << ( IRQ_Number % 32));

		} else if (IRQ_Number >= 64 && IRQ_Number < 96){
			// Configure ISER 2 register
			*NVIC_ISER2 |= ( 1 << ( IRQ_Number % 64));
	 }
   }else {
		if (IRQ_Number <= 31){
			// Configure ICER 0 register
		*NVIC_ICER0 |= (1 << IRQ_Number);
		} else if (IRQ_Number > 31 && IRQ_Number < 64){
			// Configure ICER 1 register
			*NVIC_ICER1 |= ( 1 << (IRQ_Number % 32));
		} else if (IRQ_Number > 64 && IRQ_Number < 96){
			// Configure ICER 2 register
			*NVIC_ICER2 |= ( 1 << (IRQ_Number % 64));
	 }
  }
}

void GPIO_IRQ_ProrityConfig(uint8_t IRQ_Number, uint32_t Interrupt_Prority){
	// Find the appropriate IPR register
	uint8_t iprx = IRQ_Number / 4 ;
	uint8_t iprx_section = IRQ_Number % 4 ;
	uint8_t shift_ammount = (8 * iprx_section) + NO_PR_BITS_IMPLEMENTED ;
	*(NVIC_IPR_BASE_ADDR + iprx) |= (Interrupt_Prority << shift_ammount) ;
}

void GPIO_IRQHandling(uint8_t PinNumber){
	//  Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) {

		// printf("Inside IRS handler \n") ;
		// printf("Pin # %d \n " , PinNumber) ;

		// Clear the PR register and clear the interrupt
		EXTI->PR |= (1 << PinNumber) ;
	}
}
