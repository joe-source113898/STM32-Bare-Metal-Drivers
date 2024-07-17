/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 29, 2024
 *      Author: Jose Manuel Barajas Ramirez (barajas.jose.3d@gmail.com)
 */

#include "stm32f407xx_spi_driver.h"

// SPI peripheral clock control
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pSPIx == SPI1) {
            SPI1_CLOCK_ENABLE();
        } else if (pSPIx == SPI2) {
            SPI2_CLOCK_ENABLE();
        } else if (pSPIx == SPI3) {
            SPI3_CLOCK_ENABLE();
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_CLOCK_DISABLE();
        } else if (pSPIx == SPI2) {
            SPI2_CLOCK_DISABLE();
        } else if (pSPIx == SPI3) {
            SPI3_CLOCK_DISABLE();
        }
    }
}

// Init and deinit APIs
void SPI_Init(SPI_Handle_t *pSPIHandle) {
    // Enable the clock
    SPI_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

    // First configure the SPI_CR1 register
    uint32_t tempreg = 0;

    // Configure the SPI device mode
    tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

    // Configure the bus config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        // BIDIMODE should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        // BIDIMODE should be set
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
        // BIDIMODE should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);

        // RXONLY bit should be set
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // Configure SPI clock speed
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // Configure SPI DFF mode
    tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

    // Configure SPI CPOL
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // Configure SPI CPHA
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    // Configure SPI SSM
    tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

    pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    }
}

// SPI send and receive APIs
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length) {
    while (length > 0) {
        // Check for TX buffer empty state
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16-bit data format
            // Load 16-bit data into DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            length -= 2;
            pTxBuffer += 2;
        } else {
            // Load 8-bit data into DR
            pSPIx->DR = *pTxBuffer;
            length--;
            pTxBuffer++;
        }
    }
}

void SPI_Read(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length) {
    while (length > 0) {
        // Check for RX buffer empty state
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
            // 16-bit data format
            // Read 16-bit data from DR
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            length -= 2;
            pRxBuffer += 2;
        } else {
            // Read 8-bit data from DR
            *pRxBuffer = pSPIx->DR;
            length--;
            pRxBuffer++;
        }
    }
}

// SPI get flag status
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
    if (pSPIx->SR & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

// SPI peripheral enable
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

// SPI SSI configuration
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

// SPI SSOE configuration
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

// SPI interrupt configuration
void SPI_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQ_Number <= 31) {
            // Configure ISER0 register
            *NVIC_ISER0 |= (1 << IRQ_Number);
        } else if (IRQ_Number > 31 && IRQ_Number < 64) {
            // Configure ISER1 register
            *NVIC_ISER1 |= (1 << (IRQ_Number % 32));
        } else if (IRQ_Number >= 64 && IRQ_Number < 96) {
            // Configure ISER2 register
            *NVIC_ISER2 |= (1 << (IRQ_Number % 64));
        }
    } else {
        if (IRQ_Number <= 31) {
            // Configure ICER0 register
            *NVIC_ICER0 |= (1 << IRQ_Number);
        } else if (IRQ_Number > 31 && IRQ_Number < 64) {
            // Configure ICER1 register
            *NVIC_ICER1 |= (1 << (IRQ_Number % 32));
        } else if (IRQ_Number >= 64 && IRQ_Number < 96) {
            // Configure ICER2 register
            *NVIC_ICER2 |= (1 << (IRQ_Number % 64));
        }
    }
}

uint8_t SPI_SendIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length) {
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX) {
        // Save the buffer address and length in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer; // Saving the buffer address
        pSPIHandle->TxLen = length;        // Saving the buffer length

        // Set the SPI state to busy in transmission
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // Enable the TXEIE bit to get an interrupt when the TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

        // Data transmission will happen in ISR handler
    }

    return state;
}

void SPI_CloseTransmissionIT(SPI_Handle_t *pSPIHandle) {
    // Deactivate the TXE interrupt
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

    // Reset the TX buffer
    pSPIHandle->pTxBuffer = NULL;

    // Reset the TX length to zero
    pSPIHandle->TxLen = 0;

    // Reset the TX state to SPI_READY
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReceptionIT(SPI_Handle_t *pSPIHandle) {
    // Deactivate the RXNE interrupt
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

    // Reset the RX buffer
    pSPIHandle->pRxBuffer = NULL;

    // Reset the RX length to zero
    pSPIHandle->RxLen = 0;

    // Reset the RX state to SPI_READY
    pSPIHandle->RxState = SPI_READY;
}

// SPI TXE interrupt handler
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16-bit data format
        // Load 16-bit data into DR
        pSPIHandle->pSPIx->DR = *((uint16_t*)(pSPIHandle->pTxBuffer));
        pSPIHandle->TxLen -= 2;
        pSPIHandle->pTxBuffer += 2;
    } else {
        // Load 8-bit data into DR
        pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (pSPIHandle->TxLen <= 0) {
        // TX length is zero, communication needs to be closed and inform the application that transmission is over
        // Deactivate the TXE interrupt
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

        // Reset the TX buffer
        pSPIHandle->pTxBuffer = NULL;

        // Reset the TX length to zero
        pSPIHandle->TxLen = 0;

        // Reset the TX state to SPI_READY
        pSPIHandle->TxState = SPI_READY;

        // Inform the application about the data transfer event completion
        SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

// SPI RXNE interrupt handler
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16-bit data format
        // Read 16-bit data from DR
        *((uint16_t*)(pSPIHandle->pRxBuffer)) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer += 2;
    } else {
        // Read 8-bit data from DR
        *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (pSPIHandle->RxLen <= 0) {
        // RX length is zero, communication needs to be closed and inform the application that reception is over
        // Deactivate the RXNE interrupt
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

        // Reset the RX buffer
        pSPIHandle->pRxBuffer = NULL;

        // Reset the RX length to zero
        pSPIHandle->RxLen = 0;

        // Reset the RX state to SPI_READY
        pSPIHandle->RxState = SPI_READY;

        // Inform the application about the data transfer event completion
        SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_error_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    uint8_t temp;
    // Clear the over flag

    /* Directly reading the DR register causes the OVR flag to reset but the value in the DR may be required
     * by the application, that's why we read the DR to clear the OVR flag only when SPI is not transmitting because
     * we receive data only when there is clock and clock only happens when master is transmitting */
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
        // Read operation to DR register
        temp = pSPIHandle->pSPIx->DR;

        // Read operation to SR register
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;

    // Inform the application about the error
    SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}

uint8_t SPI_ReadIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length) {
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX) {
        // Save the buffer address and length in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer; // Saving the buffer address
        pSPIHandle->RxLen = length;        // Saving the buffer length

        // Set the SPI state to busy in reception
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // Enable the RXNEIE bit to get an interrupt when the RXNE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

        // Data reception will happen in ISR handler
    }

    return state;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp; // Suppress compiler warnings
}

void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t Interrupt_Priority) {
    // Find the appropriate IPR register
    uint8_t iprx = IRQ_Number / 4;
    uint8_t iprx_section = IRQ_Number % 4;
    uint8_t shift_amount = (8 * iprx_section) + NO_PR_BITS_IMPLEMENTED;
    *(NVIC_IPR_BASE_ADDR + iprx) |= (Interrupt_Priority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
    uint8_t temp1, temp2;

    // Check for TXE
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (temp1 && temp2) {
        // Handling TXE interrupt
        spi_txe_interrupt_handle(pSPIHandle);
    }

    // Check for RXNE
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (temp1 && temp2) {
        // Handling RXNE interrupt
        spi_rxne_interrupt_handle(pSPIHandle);
    }

    // Check for OVR
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (temp1 && temp2) {
        // Handling error interrupt
        spi_error_interrupt_handle(pSPIHandle);
    }
}

/* APIs re-made */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length)
{
	// 1. Save the Tx buffer address and length information in some global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = length;

	// 2. Save bus state BUSY_IN_TX
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	// 3. Enable the TXEIE bit to get the interrupt flag whenever TXE flag is set in

	return 0;
}

__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t eventcode) {
    // This is a weak implementation of the application callback function
    if (eventcode == SPI_EVENT_TX_CMPLT) {
        // printf("Transmission completed\n");
    } else if (eventcode == SPI_EVENT_RX_CMPLT) {
        // printf("Reception completed\n");
    } else if (eventcode == SPI_EVENT_OVR_ERR) {
        // printf("OVR error has occurred, clearing now\n");
    } else if (eventcode == SPI_EVENT_CRC_ERR) {
        // printf("CRC error has occurred\n");
    }
}
