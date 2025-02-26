/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Jose Manuel Barajas Ramirez (barajas.jose.3d@gmail.com)
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPI peripheral
 * */
typedef struct{
	uint8_t	SPI_DeviceMode;	// Refer @SPI_devicemode
	uint8_t	SPI_BusConfig;	// Refer @SPI_BusConfiguration
	uint8_t SPI_SclkSpeed;	// Refer @SPI_ClockSpeed
	uint8_t SPI_DFF;		// Refer @SPI_DFF
	uint8_t SPI_CPOL;		// Refer @SPI_CPOL
	uint8_t SPI_CPHA;		// Refer @SPI_CPHA
	uint8_t SPI_SSM;		// Refer @SPI_SSM
} SPI_Config_t ;

/*
 * Handle structure for SPIx peripheral
 * */
typedef struct{
	SPI_RegDef_t   *pSPIx;	   // This holds the base address of SPIx(1 ,2,3,4) peripheral
	SPI_Config_t   SPIConfig;  // This holds the configuration of the SPIx(1, 2, 3, 4) peripheral
	uint8_t		   *pTxBuffer; // To store the App Tx buffer address
	uint8_t		   *pRxBuffer; // To store the App Rx buffer address
	uint32_t	    TxLen;	   // To store the Tx length
	uint32_t	    RxLen;	   // To store the Rx length
	uint8_t	    	TxState;   // To store the Tx state
	uint8_t	    	RxState;   // To store the Rx state
} SPI_Handle_t;

void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi) ;

// Init and DeInit APIs
void SPI_Init(SPI_Handle_t *pSPIHandle) ;
void SPI_DeInit(SPI_RegDef_t *pSPIx) ;
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) ;
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) ;
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) ;
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) ;

// SPI send and receive APIs
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_Read(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName );

// SPI interrupt configuration
void SPI_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t Interrupt_Prority);
void SPI_IRQHandling (SPI_Handle_t *pSPIHandle);

// SPI send and receive interrupt mode APIs
uint8_t SPI_SendIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReadIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);
void SPI_CloseTransmissionIT(SPI_Handle_t *pSPIHandle ) ;
void SPI_CloseRecptionIT(SPI_Handle_t *pSPIHandle ) ;

// Application event complete callback API
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t eventcode);

/* APIs re-made */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);

// SPI configuration macros
// @SPI_devicemode
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

// @SPI_busconfig
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

// @SPI_ClockConfig
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

// @SPI_DFF
#define SPI_DFF_8_BITS						0
#define SPI_DFF_16_BITS						1

// @SPI_CPOL
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

// @SPI_CPHA
#define SPI_CPHA_LOW						0
#define SPI_CPHA_HIGH						1

// @SPI_SSM
#define SPI_SSM_DI							0
#define SPI_SSM_EN							1

// SPI flag status shifts
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG					(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG					(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG					(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG					(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG					(1 << SPI_SR_FRE)

// SPI application states
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

// SPI Application events
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
