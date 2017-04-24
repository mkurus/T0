/*
#include   bootloader.h
 *
 *  Created on: 11 Eyl 2015
 *      Author: admin
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_


#define UPDATE_FIRMWARE     1
#define	NORMAL_OPERATION    0

uint32_t Calc_BlockCRC32(uint32_t u32_dataSize);

void StartNormalOperation();
void NMI_IRQHandler();
void SVC_IRQHandler();
void HardFault_IRQHandler();
void PendSV_IRQHandler();
void SysTick_Handler();
void SPI0_IRQHandler();
void SPI1_IRQHandler();
void UART0_IRQHandler();
void UART1_IRQHandler();
void UART2_IRQHandler();
void I2C1_IRQHandler();
void I2C0_IRQHandler();
void I2C2_IRQHandler();
void I2C3_IRQHandler();
void SCT_IRQHandler();
void MRT_IRQHandler();
void CMP_IRQHandler();
void WDT_IRQHandler();
void BOD_IRQHandler();
void FLASH_IRQHandler();
void WKT_IRQHandler();
void ADC_SEQA_IRQHandler();
void ADC_SEQB_IRQHandler();
void ADC_THCMP_IRQHandler();
void ADC_OVR_IRQHandler();
void DMA_IRQHandler();
void PIN_INT0_IRQHandler();
void PIN_INT1_IRQHandler();
void PIN_INT2_IRQHandler();
void PIN_INT3_IRQHandler();
void PIN_INT4_IRQHandler();
void PIN_INT5_IRQHandler();
void PIN_INT6_IRQHandler();
void PIN_INT7_IRQHandler();
//*****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//
//*****************************************************************************
void ResetISR(void);
void NMI_Handler_Jump_Vector(void);
void HardFault_Handler_Jump_Vector(void);
void SVC_Handler_Jump_Vector(void);
void PendSV_Handler_Jump_Vector(void);
void SysTick_Handler_Jump_Vector(void);

void IntDefaultHandler(void);


//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
//*****************************************************************************
void SPI0_IRQHandler_Jump_Vector(void);
void SPI1_IRQHandler_Jump_Vector(void);
void UART0_IRQHandler_Jump_Vector(void);
void UART1_IRQHandler_Jump_Vector(void);
void UART2_IRQHandler_Jump_Vector(void);
void I2C1_IRQHandler_Jump_Vector(void);
void I2C0_IRQHandler_Jump_Vector(void);
void SCT_IRQHandler_Jump_Vector(void);
void MRT_IRQHandler_Jump_Vector(void);
void CMP_IRQHandler_Jump_Vector(void);
void WDT_IRQHandler_Jump_Vector(void);
void BOD_IRQHandler_Jump_Vector(void);
void FLASH_IRQHandler_Jump_Vector(void);
void WKT_IRQHandler_Jump_Vector(void);
void ADC_SEQA_IRQHandler_Jump_Vector(void);
void ADC_SEQB_IRQHandler_Jump_Vector(void) ;
void ADC_THCMP_IRQHandler_Jump_Vector(void);
void ADC_OVR_IRQHandler_Jump_Vector(void);
void DMA_IRQHandler_Jump_Vector(void) ;
void I2C2_IRQHandler_Jump_Vector(void) ;
void I2C3_IRQHandler_Jump_Vector(void) ;
void PIN_INT0_IRQHandler_Jump_Vector(void);
void PIN_INT1_IRQHandler_Jump_Vector(void) ;
void PIN_INT2_IRQHandler_Jump_Vector(void) ;
void PIN_INT3_IRQHandler_Jump_Vector(void);
void PIN_INT4_IRQHandler_Jump_Vector(void) ;
void PIN_INT5_IRQHandler_Jump_Vector(void) ;
void PIN_INT6_IRQHandler_Jump_Vector(void) ;
void PIN_INT7_IRQHandler_Jump_Vector(void);

void  *memset_boot(void *buffer, int c, int len);
#endif /* BOOTLOADER_H_ */
