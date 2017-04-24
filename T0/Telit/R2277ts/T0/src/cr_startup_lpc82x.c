//*****************************************************************************
// LPC82x Microcontroller Startup code for use with LPCXpresso IDE
//
// Version : 141204
//*****************************************************************************
//
// Copyright(C) NXP Semiconductors, 2014
// All rights reserved.
//
// Software that is described herein is for illustrative purposes only
// which provides customers with programming information regarding the
// LPC products.  This software is supplied "AS IS" without any warranties of
// any kind, and NXP Semiconductors and its licensor disclaim any and
// all warranties, express or implied, including all implied warranties of
// merchantability, fitness for a particular purpose and non-infringement of
// intellectual property rights.  NXP Semiconductors assumes no responsibility
// or liability for the use of the software, conveys no license or rights under any
// patent, copyright, mask work right, or any other intellectual property rights in
// or to any products. NXP Semiconductors reserves the right to make changes
// in the software without notification. NXP Semiconductors also makes no
// representation or warranty that such application will be suitable for the
// specified use without further testing or modification.
//
// Permission to use, copy, modify, and distribute this software and its
// documentation is hereby granted, under NXP Semiconductors' and its
// licensor's relevant copyrights in the software, without fee, provided that it
// is used in conjunction with NXP Semiconductors microcontrollers.  This
// copyright, permission, and disclaimer notice must appear in all copies of
// this code.
//*****************************************************************************

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#include "board.h"
#include "timer.h"
#include "chip.h"
#include "utils.h"
#include "spi.h"
#include "sst25.h"
#include "bootloader.h"
#include <string.h>
#include <stdlib.h>
//*****************************************************************************

#if defined (__USE_CMSIS) || defined (__USE_LPCOPEN)
// Declaration of external SystemInit function
extern void SystemInit(void);
void Board_LED_Init_Bootloader();
void Blink5();
void Delay_Bootloader();
static inline void EnablePeriphClock_Bootloader(CHIP_SYSCTL_CLOCK_T clk);
static inline void DisablePeriphClock_Bootloader(CHIP_SYSCTL_CLOCK_T clk);
static inline void Clock_SetUARTClockDiv_Bootloader(uint32_t div);
static inline void SYSCTL_AssertPeriphReset_Bootloader(CHIP_SYSCTL_PERIPH_RESET_T periph);
static inline void SYSCTL_DeassertPeriphReset_Bootloader(CHIP_SYSCTL_PERIPH_RESET_T periph);
static inline void SYSCTL_PeriphReset_Bootloader(CHIP_SYSCTL_PERIPH_RESET_T periph);
static inline void SYSCTL_SetUSARTFRGMultiplier_Bootloader(uint8_t mult);
static inline void SYSCTL_SetUSARTFRGDivider_Bootloader(uint8_t div);
static inline void SPI_Init_Bootloader();
static inline void SPI_ConfigureSPI_Bootloader(LPC_SPI_T *pSPI, uint32_t config);
static inline void SPI_ClearCFGRegBits_Bootloader(LPC_SPI_T *pSPI, uint32_t bits);
static inline void SPI_SetCFGRegBits_Bootloader(LPC_SPI_T *pSPI, uint32_t bits);
static inline void SPI_ClearStatus_Bootloader(LPC_SPI_T *pSPI, uint32_t Flag);
static inline void SPI_SetControlInfo_Bootloader(LPC_SPI_T *pSPI, uint8_t Flen, uint32_t Flag);
static inline uint32_t SPI_GetStatus_Bootloader(LPC_SPI_T *pSPI);
static inline void SPI_FlushFifos_Bootloader();
static inline void SPI_SendLastFrame_Bootloader(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize);
static inline void SPI_SendMidFrame_Bootloader(LPC_SPI_T *pSPI, uint16_t Data);
static inline void SPI_Enable_Bootloader(LPC_SPI_T *pSPI);
static inline void SPI_Disable_Bootloader(LPC_SPI_T *pSPI);
static inline uint16_t SPI_ReceiveFrame_Bootloader(LPC_SPI_T *pSPI);
static inline void NVIC_SystemReset_Bootloader(void);
static inline void iap_entry_bootloader(unsigned int cmd_param[], unsigned int status_result[]);
STATIC INLINE void Chip_GPIO_SetPinDIROutput_Bootloader(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin);
STATIC INLINE void Chip_GPIO_SetPinState_Bootloader(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting);
void Board_LED_Toggle_Bootloader(uint8_t LEDNumber);
STATIC INLINE void Chip_GPIO_SetPinToggle_Bootloader(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin);
static uint32_t Clock_GetSystemClockRate_Bootloader(void);
static void Initialize_SPI_Interface();

static void SWM_MovablePinAssign_Bootloader(CHIP_SWM_PIN_MOVABLE_T movable, uint8_t pin);
static uint32_t SPIM_SetClockRate_Bootloader(LPC_SPI_T *, uint32_t);
static uint32_t Chip_SPIM_GetClockRate_Bootloader(LPC_SPI_T *);

__attribute__( ( always_inline ) ) static inline void __disable_irq_bootloader();
__attribute__( ( always_inline ) ) static inline void __enable_irq_bootloader();

uint8_t IAP_PreSectorForReadWrite_Boot(uint32_t strSector, uint32_t endSector);
uint8_t IAP_EraseSector_Boot(uint32_t strSector, uint32_t endSector);
uint8_t IAP_CopyRamToFlash_Boot(uint32_t dstAdd, uint32_t *srcAdd, uint32_t byteswrt);
void Chip_SetupIrcClocking_Bootloader(void);
void SetupPinMuxing_Bootloader();
void UART0_Init_Bootloader();
void SetupMuxing_Bootloader(void);
uint8_t SystemInit_Bootloader(void);
void Init_SST25_Bootloader();
void Chip_Clock_SetSysClockDiv_Bootloader(uint32_t div);
void Chip_Clock_SetMainClockSource_Bootloader(CHIP_SYSCTL_MAINCLKSRC_T src);
void Chip_Clock_SetUARTClockDiv_Bootloader(uint32_t div);
void SetupDebugPort_Bootloader();
void putLineUART_Bootloader(char *send_data, uint32_t data_length);
uint32_t Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);
uint32_t Clock_GetSystemClockRate_Bootloader(void);
int HexToStr_Bootloader(char *, uint32_t val);
void SPI_Send_Data_Bootloader(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);
void SPI_Receive_Data_Bootloader(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup);

void SST25_EraseSector_Bootloader(uint32_t sector_no);
void SST25_WriteSR_Bootloader(uint8_t data);
void SST25_WriteEnable_Bootloader();
void SST25_WriteByte_Bootloader(uint32_t address, uint8_t data);
uint32_t SST25_WriteArray_Bootloader(char *array, uint16_t length, uint32_t u32_flashAddress);
void SST25_Read_Bootloader(uint32_t address, uint16_t length, char *out_buffer);
void SST25_ReadSR_Bootloader();
bool SST25_ReadBUSY_Bootloader();
void SST25_EnableWSR_Bootloader();
void SST25_Jedec_ID_Read_Bootloader();
void ErrorUART_Bootloader(void);
void CheckOperatingMode();
void crcInit(void);
void Init_IAP();
void UpdateFirmware();
void EraseImageSignature();
void  *memset_boot(void *buffer, int c, int len);

uint32_t crcFast(uint8_t const *message, int32_t nBytes);
static uint32_t reflect(uint32_t data, uint8_t nBits);
#endif

// Patch the AEABI integer divide functions to use MCU's romdivide library
#ifdef __USE_ROMDIVIDE
// Location in memory that holds the address of the ROM Driver table
#define PTR_ROM_DRIVER_TABLE ((unsigned int *)(0x1FFF1FF8))

// Variables to store addresses of idiv and udiv functions within MCU ROM
//unsigned int *pDivRom_idiv;
//unsigned int *pDivRom_uidiv;
unsigned int *pDivRom_idiv __attribute__ ((used, section (".romDiv_Vars")));
unsigned int *pDivRom_uidiv __attribute__ ((used, section (".romDiv_Vars")));
#endif

/* Size of each sector */
#define ONCHIP_FLASH_SECTOR_SIZE             1024
/* Last sector address */
#define USER_CODE_START_ADDR    ONCHIP_FLASH_SECTOR_SIZE * 2

/* LAST SECTOR */
#define IAP_LAST_SECTOR         31
/* LAST SECTOR */
#define IAP_FIRST_SECTOR        2

/* Number elements in array */
#define ARRAY_ELEMENTS              (ONCHIP_FLASH_SECTOR_SIZE / sizeof(uint32_t))

#define MAGICWORD_EXT_FLASH_ADDR     251 * 4096

#define NEW_FIRMWARE_MAGICWORD             0xAA55AA55

#define ON_RESET_CORE_CLOCK          12000000

/* Data array to write to flash */
//static uint32_t src_iap_array_data[ARRAY_ELEMENTS];

#define SPI_TX_BUFFER_SIZE     128
#define SPI_RX_BUFFER_SIZE     1028
/* UART handle and memory for ROM API */
UART_HANDLE_T *uartHandle          __attribute__ ((section (".bootloader_ram")));

/* Use a buffer size larger than the expected return value of
   uart_get_mem_size() for the static UART handle type */
uint32_t uartHandleMEM[0x40]       __attribute__ ((section (".bootloader_ram")));

/* SPI transfer setup */
SPI_DATA_SETUP_T XfSetup           __attribute__ ((section (".bootloader_ram")));

/* SPI Tx buffer */
uint8_t TxBuf[SPI_TX_BUFFER_SIZE]  __attribute__ ((section (".bootloader_ram")));

/* SPI Rx buffer */
uint8_t RxBuf[SPI_RX_BUFFER_SIZE]  __attribute__ ((section (".bootloader_ram")));

uint32_t crcTable[256]             __attribute__ ((section (".bootloader_ram")));
UART_CONFIG_T cfg                  __attribute__ ((section (".bootloader_ram")));
uint32_t src_iap_array_data1[ONCHIP_FLASH_SECTOR_SIZE / sizeof(uint32_t)] __attribute__ ((section (".bootloader_ram")));
char str1[] __attribute__  ((section(".boot_string"))) = "Writing sector...";
char str2[] __attribute__  ((section(".boot_string"))) = "Prepared";
char str3[] __attribute__  ((section(".boot_string"))) = "Erased";
char str4[] __attribute__  ((section(".boot_string"))) = "Written";
char strBooting [] __attribute__  ((section(".boot_string"))) = "Booting";
char asm_dsb[] __attribute__  ((section(".boot_string"))) = "dsb";
char strPtr[]    __attribute__  ((section(".boot_string"))) = "0123456789ABCDEF";

void ResetISR(void);

//*****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
//*****************************************************************************
extern void _vStackTop(void);
extern uint32_t _vStackBottom;
//*****************************************************************************
//
// The vector table.
// This relies on the linker script to place at correct location in memory.
//
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM0plus
    &_vStackTop, // The initial stack pointer
    ResetISR,                               // The reset handler
    NMI_Handler_Jump_Vector,                // The NMI handler
    HardFault_Handler_Jump_Vector,          // The hard fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    SVC_Handler_Jump_Vector,                // SVCall handler
    0,                                      // Reserved
    0,                                      // Reserved
    PendSV_Handler_Jump_Vector,             // The PendSV handler
    SysTick_Handler_Jump_Vector,           	// The SysTick handler

    // Chip Level - LPC82x
    SPI0_IRQHandler_Jump_Vector,             // SPI0 controller
    SPI1_IRQHandler_Jump_Vector,             // SPI1 controller
    0,                                       // Reserved
    UART0_IRQHandler_Jump_Vector,            // UART0
    UART1_IRQHandler_Jump_Vector,            // UART1
    UART2_IRQHandler_Jump_Vector,            // UART2
    0,                                       // Reserved
    I2C1_IRQHandler_Jump_Vector,             // I2C1 controller
    I2C0_IRQHandler_Jump_Vector,             // I2C0 controller
    SCT_IRQHandler_Jump_Vector,              // Smart Counter Timer
    MRT_IRQHandler_Jump_Vector,              // Multi-Rate Timer
    CMP_IRQHandler_Jump_Vector,              // Comparator
    WDT_IRQHandler_Jump_Vector,              // Watchdog
    BOD_IRQHandler_Jump_Vector,              // Brown Out Detect
    FLASH_IRQHandler_Jump_Vector,            // Flash Interrupt
    WKT_IRQHandler_Jump_Vector,              // Wakeup timer
    ADC_SEQA_IRQHandler_Jump_Vector,         // ADC sequence A completion
    ADC_SEQB_IRQHandler_Jump_Vector,         // ADC sequence B completion
    ADC_THCMP_IRQHandler_Jump_Vector,        // ADC threshold compare
    ADC_OVR_IRQHandler_Jump_Vector,          // ADC overrun
    DMA_IRQHandler_Jump_Vector,              // DMA
    I2C2_IRQHandler_Jump_Vector,             // I2C2 controller
    I2C3_IRQHandler_Jump_Vector,             // I2C3 controller
    0,                                       // Reserved
    PIN_INT0_IRQHandler_Jump_Vector,         // PIO INT0
    PIN_INT1_IRQHandler_Jump_Vector,         // PIO INT1
    PIN_INT2_IRQHandler_Jump_Vector,         // PIO INT2
    PIN_INT3_IRQHandler_Jump_Vector,         // PIO INT3
    PIN_INT4_IRQHandler_Jump_Vector,         // PIO INT4
    PIN_INT5_IRQHandler_Jump_Vector,         // PIO INT5
    PIN_INT6_IRQHandler_Jump_Vector,         // PIO INT6
    PIN_INT7_IRQHandler_Jump_Vector,         // PIO INT7
}; /* End of g_pfnVectors */


//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
extern unsigned int _bootloader_data_start;
extern unsigned int _bootloader_data_end;

//__attribute__((optimize("-O0")))
__attribute__ ((section(".bootloader_func")))
void ResetISR(void) {

#ifdef __USE_ROMDIVIDE
    // Get address of Integer division routines function table in ROM
    unsigned int *div_ptr = (unsigned int *)((unsigned int *)*(PTR_ROM_DRIVER_TABLE))[4];
    // Get addresses of integer divide routines in ROM
    // These address are then used by the code in aeabi_romdiv_patch.s
    pDivRom_idiv = (unsigned int *)div_ptr[0];
    pDivRom_uidiv = (unsigned int *)div_ptr[1];

#endif
    Board_LED_Init_Bootloader();

    Blink5();
     if(SystemInit_Bootloader() == NORMAL_OPERATION){
    	 StartNormalOperation();
    }
     else{
    	 UpdateFirmware();
    	 EraseImageSignature();
    	 NVIC_SystemReset_Bootloader();
     }
    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}
/****************************************************************************/
__attribute__ ((section(".bootloader_func")))
void EraseImageSignature()
{
	/* erase firmware signature */
    SST25_EraseSector_Bootloader(FIRMWARE_MAGICWORD_ADDRESS/SST25_SECTOR_SIZE);
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
void DumpFlashPage()
{
/*	uint32_t  *ptr;
	int  h;
	char temp[16];

	memset_boot(temp, 0, 16);
	//	ptr = src_iap_array_data1;
		for (h = 0; h < 256; h++){
			HexToStr_Bootloader(temp , *ptr++);
			putLineUART_Bootloader(temp, 8);
		    putLineUART_Bootloader("\r" ,1);
		}

		putLineUART_Bootloader("\n*******\n" ,20);

		ptr = 2 * ONCHIP_FLASH_SECTOR_SIZE;
		for (h = 0; h < 256; h++){
			HexToStr_Bootloader(temp , *ptr++);
			putLineUART_Bootloader(temp, 8);
			putLineUART_Bootloader("\r" ,1);
		}*/
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
int HexToStr_Bootloader(char *dest, uint32_t val)
{
	int i, ret = 0;
	for (i = 0; i < sizeof(val) * 2; i ++) {
		int idx = val & 0xF;
		dest[7 - i] = strPtr[idx];
		val >>= 4;
		if (idx)
			ret = i;
	}
	return ret + 1;
}
/****************************************************************************/
__attribute__ ((section(".bootloader_func")))
void Blink5()
{
	int i = 0;
	while(i <= 2){
		Board_LED_Toggle_Bootloader(8);
	    Delay_Bootloader();
	    i++;
	}
}
/*******************************************************************************/
__attribute__ ((section(".bootloader_func")))
void Board_LED_Init_Bootloader()
{
	Chip_GPIO_SetPinDIROutput_Bootloader(LPC_GPIO_PORT, 0, 8);
	Chip_GPIO_SetPinState_Bootloader(LPC_GPIO_PORT, 0, 8, true);
}
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
STATIC INLINE void Chip_GPIO_SetPinDIROutput_Bootloader(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO->DIRSET[port] = 1UL << pin;

}
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
STATIC INLINE void Chip_GPIO_SetPinState_Bootloader(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting)
{
	pGPIO->B[port][pin] = setting;
}
/******************************************************************************/

/* Toggles the current state of a board LED */
__attribute__ ((section(".bootloader_func")))
void Board_LED_Toggle_Bootloader(uint8_t LEDNumber)
{
	Chip_GPIO_SetPinToggle_Bootloader(LPC_GPIO_PORT, 0, 8);
}
/*************************************************************************************/
__attribute__ ((section(".bootloader_func")))
STATIC INLINE void Chip_GPIO_SetPinToggle_Bootloader(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin)
{
	pGPIO->NOT[port] = (1 << pin);
}
/*******************************************************************************/
__attribute__ ((section(".bootloader_func")))
extern void __dsb();

static inline void NVIC_SystemReset_Bootloader(void)
{
  __ASM volatile ("dsb");
	//__dsb();    /* Ensure all outstanding memory accesses included*/
                                                             /*     buffered write are completed before reset */
  SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
                 SCB_AIRCR_SYSRESETREQ_Msk);
   __ASM volatile ("dsb");   /* Ensure completion of memory access */
   Blink5();
  //__dsb();
  while(1);                                                    /* wait until reset */
} /* Ensure completion of memory access */
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
void UpdateFirmware(void)
{
	uint32_t u32_dataOffset;
	uint32_t  i;

	u32_dataOffset = FIRMWARE_ADDRESS + (IAP_FIRST_SECTOR * ONCHIP_FLASH_SECTOR_SIZE);

	for(i =  IAP_FIRST_SECTOR; i <= IAP_LAST_SECTOR; i++){

	//	putLineUART_Bootloader(str1, sizeof(str1) + 1);
		SST25_Read_Bootloader(u32_dataOffset, ONCHIP_FLASH_SECTOR_SIZE, (char *)src_iap_array_data1);
	//	SST25_Read_Bootloader(u32_dataOffset, ONCHIP_FLASH_SECTOR_SIZE, (char *)src_iap_array_data2);

		__disable_irq_bootloader();
		IAP_PreSectorForReadWrite_Boot(i, i);
	//	__enable_irq_bootloader();

		//__disable_irq_bootloader();
		IAP_EraseSector_Boot(i, i);
	//	__enable_irq_bootloader();

	//__disable_irq_bootloader();
		IAP_PreSectorForReadWrite_Boot(i, i);
	//	__enable_irq_bootloader();

	//	__disable_irq_bootloader();
	     IAP_CopyRamToFlash_Boot(i * ONCHIP_FLASH_SECTOR_SIZE, src_iap_array_data1, ONCHIP_FLASH_SECTOR_SIZE);
		__enable_irq_bootloader();

		//	Send_String();
	//	DumpFlashPage();
		// Board_LED_Toggle_Bootloader(0);
		Blink5();
		u32_dataOffset += ONCHIP_FLASH_SECTOR_SIZE;

	}

}
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
__attribute__( ( always_inline ) ) static inline void __disable_irq_bootloader()
{
	__ASM volatile ("cpsid i" : : : "memory");
}
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
__attribute__( ( always_inline ) ) static inline void __enable_irq_bootloader()
{
	__ASM volatile ("cpsie i" : : : "memory");
}
/******************************************************************************/
/* Set up and initialize hardware prior to call to main */
__attribute__ ((section(".bootloader_func")))
uint8_t SystemInit_Bootloader(void)
{
    char strBuffer[32];
	uint32_t imageSize;
	uint32_t imageCRC;
	uint32_t calcCRC;

	/* clear memory used by bootloader app */
	memset_boot(&_bootloader_data_start, 0, &_bootloader_data_end - &_bootloader_data_start);
	EnablePeriphClock_Bootloader(SYSCTL_CLOCK_IOCON);

	SetupPinMuxing_Bootloader();
	Initialize_SPI_Interface();
	Init_SST25_Bootloader();
	memset_boot(strBuffer, 0, sizeof(strBuffer));
	SST25_Read_Bootloader(FIRMWARE_MAGICWORD_ADDRESS, 4, (char *)&imageSize);
	SST25_Read_Bootloader(FIRMWARE_MAGICWORD_ADDRESS + 4, 4, (char *)&imageCRC);
	if(imageCRC != 0xFFFFFFFF && imageSize != 0xFFFFFFFF){
		calcCRC = Calc_BlockCRC32(imageSize);
		if(calcCRC == imageCRC)
			return UPDATE_FIRMWARE;
	}
	return NORMAL_OPERATION;
}
/*********************************************************************/
__attribute__ (( section(".bootloader_func")))
void Send_String()
{
	 char dum[10];
		   dum[0] = 0x31;
		   dum[1] = 0x32;

		putLineUART_Bootloader(dum, 2);
}
/********************************************************************/
/* Sets up system pin muxing */
__attribute__ (( section(".bootloader_func")))
void SetupPinMuxing_Bootloader(void)
{
	/* Enable IOCON and Switch Matrix clocks */
	EnablePeriphClock_Bootloader(SYSCTL_CLOCK_SWM);

	Clock_SetUARTClockDiv_Bootloader(1);	/* divided by 1 */

	/* UARt pin mux */
	SWM_MovablePinAssign_Bootloader(SWM_U0_TXD_O, 4);
//	SWM_MovablePinAssign_Bootloader(SWM_U0_RXD_I, 0);

	/* SPI pin mux */
	SWM_MovablePinAssign_Bootloader(SWM_SPI0_SSEL0_IO, 26);
	SWM_MovablePinAssign_Bootloader(SWM_SPI0_SCK_IO, 24);
	SWM_MovablePinAssign_Bootloader(SWM_SPI0_MISO_IO, 25);
	SWM_MovablePinAssign_Bootloader(SWM_SPI0_MOSI_IO, 15);

	/* Disable the clock to the Switch Matrix to save power */
	DisablePeriphClock_Bootloader(SYSCTL_CLOCK_SWM);

}
/****************************************************************/
/* Initialize the UART peripheral */
__attribute__ ((section(".bootloader_func")))
void UART0_Init_Bootloader()
{
	/* Enable USART clock */
	EnablePeriphClock_Bootloader(SYSCTL_CLOCK_UART0);
	/* Peripheral reset control to USART0 */
	SYSCTL_PeriphReset_Bootloader(RESET_USART0);

}
/****************************************************************/
/* Set main system clock source */
__attribute__ ((section(".bootloader_func")))
void Chip_Clock_SetMainClockSource_Bootloader(CHIP_SYSCTL_MAINCLKSRC_T src)
{
	LPC_SYSCTL->MAINCLKSEL  = (uint32_t) src;

    /* sequence a 0 followed by 1 to update MAINCLK source selection */
	LPC_SYSCTL->MAINCLKUEN  = 0;
	LPC_SYSCTL->MAINCLKUEN  = 1;
}
/*********************************************************************/
/* Return main clock rate */
__attribute__ ((section(".bootloader_func")))
static inline uint32_t Clock_GetMainClockRate_Bootloader(void)
{
	return SYSCTL_IRC_FREQ;

}
/**********************************************************************/
/* Setup UART handle and parameters */
__attribute__ ((section(".bootloader_func")))
void SetupDebugPort_Bootloader()
{
	 uint32_t frg_mult;


	 cfg.sys_clk_in_hz = 0;
	 cfg.baudrate_in_hz = 115200;
	 cfg.config = 1;
	 cfg.sync_mod = 0;
	 cfg.error_en = NO_ERR_EN;

		/* 115.2KBPS, 8N1, ASYNC mode, no errors, clock filled in later */
	//	 cfg = {
	//		0,				/* U_PCLK frequency in Hz */
	//		115200,			/* Baud Rate in Hz */
	//		1,				/* 8N1 */
	//		0,				/* Asynchronous Mode */
	//		NO_ERR_EN		/* Enable No Errors */
	//	};

		/* Perform a sanity check on the storage allocation */
		if (LPC_UARTD_API->uart_get_mem_size() > sizeof(uartHandleMEM)) {
			/* Example only: this should never happen and probably isn't needed for
			   most UART code. */

			ErrorUART_Bootloader();
		}

		/* Setup the UART handle */
		uartHandle = LPC_UARTD_API->uart_setup((uint32_t) LPC_USART0, (uint8_t *) &uartHandleMEM);
		if (uartHandle == NULL) {

			ErrorUART_Bootloader();
		}

		/* Need to tell UART ROM API function the current UART peripheral clock
		     speed */
		cfg.sys_clk_in_hz = Clock_GetSystemClockRate_Bootloader();

		/* Initialize the UART with the configuration parameters */
		frg_mult = LPC_UARTD_API->uart_init(uartHandle, &cfg);
		if (frg_mult) {
		    SYSCTL_SetUSARTFRGDivider_Bootloader(0xFF);	/* value 0xFF should be always used */
			SYSCTL_SetUSARTFRGMultiplier_Bootloader(frg_mult);
		}

}
/*****************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SYSCTL_SetUSARTFRGMultiplier_Bootloader(uint8_t mult)
{
	LPC_SYSCTL->UARTFRGMULT = (uint32_t) mult;
}
/*****************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SYSCTL_SetUSARTFRGDivider_Bootloader(uint8_t div)
{
	LPC_SYSCTL->UARTFRGDIV = (uint32_t) div;
}
/*****************************************************************/
/* Turn on LED to indicate an error */
__attribute__ ((section(".bootloader_func")))
void ErrorUART_Bootloader(void)
{
	//Board_LED_Set(0, true);
	while (1)  {}
}
/*****************************************************************/
__attribute__ ((section(".bootloader_func")))
void Delay_Bootloader()
{
	volatile uint32_t i;
	volatile uint32_t k =0;
	for(i =0; i< 200000; i++)
		k++;


}
/******************************************************************/
/* Return system clock rate */
__attribute__ ((section(".bootloader_func")))
static uint32_t Clock_GetSystemClockRate_Bootloader(void)
{
	/* No point in checking for divide by 0 */
	return Clock_GetMainClockRate_Bootloader() / (LPC_SYSCTL->SYSAHBCLKDIV & ~SYSCTL_SYSAHBCLKDIV_RESERVED);
}
//*******************************************************************/
/* Send a string on the UART terminated by a NULL character using
   polling mode. */
__attribute__ ((section(".bootloader_func")))
void putLineUART_Bootloader(char *send_data, uint32_t data_length)
{
	UART_PARAM_T param;

	param.buffer =  (uint8_t *)send_data;
	param.size = data_length;

	/* Polling mode, do not append CR/LF to sent data */
	param.transfer_mode = TX_MODE_SZERO_SEND_CRLF;
	param.driver_mode = DRIVER_MODE_POLLING;
	/* Transmit the data */
	if (LPC_UARTD_API->uart_put_line(uartHandle, &param)) {

		ErrorUART_Bootloader();
	}
/*	LPC_UARTD_API->uart_put_char(uartHandle, 'A');*/

}
/*********************************************************************/
__attribute__ ((section(".bootloader_func")))
static void Initialize_SPI_Interface()
{
	SPI_Init_Bootloader();

  /* Call to initialize first SPI controller for mode0, master mode, MSB first */
	SPI_ConfigureSPI_Bootloader(LPC_SPI0,SPI_MODE_MASTER |	/* Enable master mode */
						        SPI_CLOCK_CPHA0_CPOL0 |	           /* Set Clock polarity to 0 */
						        SPI_CFG_MSB_FIRST_EN |              /* Enable MSB first option */
						        SPI_CFG_SPOL_LO);	               /* Chipselect is active low */

  /* Setup master clock rate, slave clock doesn't need to be setup */
    SPIM_SetClockRate_Bootloader(LPC_SPI0, 100000);
    SPI_FlushFifos_Bootloader(LPC_SPI0);
	XfSetup.pTx = TxBuf;
	XfSetup.pRx = RxBuf;
	XfSetup.DataSize = 8;
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_ConfigureSPI_Bootloader(LPC_SPI_T *pSPI, uint32_t config)
{
	SPI_ClearCFGRegBits_Bootloader(pSPI, SPI_CFG_MASTER_EN | SPI_CFG_LSB_FIRST_EN |
			            SPI_CFG_CPHA_SECOND | SPI_CFG_CPOL_HI);
    SPI_SetCFGRegBits_Bootloader(pSPI, config);

	/* Deassert all chip selects, only in master mode */
	pSPI->TXCTRL = SPI_TXDATCTL_DEASSERT_ALL;
}
/****************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_ClearCFGRegBits_Bootloader(LPC_SPI_T *pSPI, uint32_t bits)
{
	/* Update CFG register with only selected bits disabled */
	pSPI->CFG = ~bits & (pSPI->CFG & SPI_CFG_BITMASK);
}
/****************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_SetCFGRegBits_Bootloader(LPC_SPI_T *pSPI, uint32_t bits)
{
	/* Update CFG register with only selected bits disabled */
	pSPI->CFG = bits | (pSPI->CFG & SPI_CFG_BITMASK);
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_FlushFifos_Bootloader()
{
	SPI_Disable_Bootloader(LPC_SPI0);
	SPI_Enable_Bootloader(LPC_SPI0);
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline  void SPI_Enable_Bootloader(LPC_SPI_T *pSPI)
{
	SPI_SetCFGRegBits_Bootloader(pSPI, SPI_CFG_SPI_EN);
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_Disable_Bootloader(LPC_SPI_T *pSPI)
{
	Chip_SPI_ClearCFGRegBits(pSPI, SPI_CFG_SPI_EN);
}

/***************************************************************************/
/* Set SPI master bit rate */
__attribute__ ((section(".bootloader_func")))
static uint32_t SPIM_SetClockRate_Bootloader(LPC_SPI_T *pSPI, uint32_t rate)
{
	uint32_t baseClock, div;

	/* Get peripheral base clock rate */
	baseClock = Clock_GetSystemClockRate_Bootloader();

	/* Compute divider */
	div = baseClock / rate;

	/* Limit values */
	if (div == 0) {
		div = 1;
	}
	else if (div > 0x10000) {
		div = 0x10000;
	}
	pSPI->DIV = div - 1;

	return Chip_SPIM_GetClockRate_Bootloader(pSPI);
}
/****************************************************************/
/* Get SPI master bit rate */
__attribute__ ((section(".bootloader_func")))
static uint32_t Chip_SPIM_GetClockRate_Bootloader(LPC_SPI_T *pSPI)
{
	return Clock_GetSystemClockRate_Bootloader() / ((pSPI->DIV & ~SPI_DIV_RESERVED) + 1);
}
/*****************************************************************/
/*Send and Receive SPI Data  */
__attribute__ ((section(".bootloader_func")))
uint32_t Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup)
{
	uint32_t Status;
	/* Clear status */
	SPI_ClearStatus_Bootloader(pSPI, SPI_STAT_CLR_RXOV | SPI_STAT_CLR_TXUR | SPI_STAT_CLR_SSA | SPI_STAT_CLR_SSD);
	SPI_SetControlInfo_Bootloader(pSPI, pXfSetup->DataSize, SPI_TXCTL_ASSERT_SSEL | SPI_TXCTL_EOF);
	pXfSetup->TxCnt = pXfSetup->RxCnt = 0;
	while ((pXfSetup->TxCnt < pXfSetup->Length) ||
		   (pXfSetup->RxCnt < pXfSetup->Length)) {
		Status = SPI_GetStatus_Bootloader(pSPI);

		/* In case of TxReady */
		if ((Status & SPI_STAT_TXRDY) && (pXfSetup->TxCnt < pXfSetup->Length)) {
			SPI_Send_Data_Bootloader(pSPI, pXfSetup);
		}

		/*In case of Rx ready */
		if ((Status & SPI_STAT_RXRDY) && (pXfSetup->RxCnt < pXfSetup->Length)) {
			SPI_Receive_Data_Bootloader(pSPI, pXfSetup);
		}
	}
	/* Check error */
	if (SPI_GetStatus_Bootloader(pSPI) & (SPI_STAT_CLR_RXOV | SPI_STAT_CLR_TXUR)) {
		return 0;
	}
	return pXfSetup->TxCnt;
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline uint32_t SPI_GetStatus_Bootloader(LPC_SPI_T *pSPI)
{
	return pSPI->STAT & ~SPI_STAT_RESERVED;
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_SetControlInfo_Bootloader(LPC_SPI_T *pSPI, uint8_t Flen, uint32_t Flag)
{
	pSPI->TXCTRL = Flag | SPI_TXDATCTL_FLEN(Flen - 1);
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_ClearStatus_Bootloader(LPC_SPI_T *pSPI, uint32_t Flag)
{
	pSPI->STAT = Flag;
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SPI_Send_Data_Bootloader(LPC_SPI_T *pSPI,
						  SPI_DATA_SETUP_T *pXfSetup)
{
	if (pXfSetup->TxCnt == (pXfSetup->Length - 1)) {
		SPI_SendLastFrame_Bootloader(pSPI, pXfSetup->pTx[pXfSetup->TxCnt], pXfSetup->DataSize);
	}
	else {
		SPI_SendMidFrame_Bootloader(pSPI, pXfSetup->pTx[pXfSetup->TxCnt]);
	}

	pXfSetup->TxCnt++;
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_SendMidFrame_Bootloader(LPC_SPI_T *pSPI, uint16_t Data)
{
	pSPI->TXDAT = SPI_TXDAT_DATA(Data);
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_SendLastFrame_Bootloader(LPC_SPI_T *pSPI, uint16_t Data, uint8_t DataSize)
{
	pSPI->TXDATCTL = SPI_TXDATCTL_ASSERT_SSEL | SPI_TXDATCTL_EOF | SPI_TXDATCTL_EOT |
					 SPI_TXDATCTL_FLEN(DataSize - 1) | SPI_TXDATCTL_DATA(Data);
}
/*************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SPI_Receive_Data_Bootloader(LPC_SPI_T *pSPI,
							 SPI_DATA_SETUP_T *pXfSetup)
{
	pXfSetup->pRx[pXfSetup->RxCnt] = SPI_ReceiveFrame_Bootloader(pSPI);
	pXfSetup->RxCnt++;
}
/**************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline uint16_t SPI_ReceiveFrame_Bootloader(LPC_SPI_T *pSPI)
{
	return SPI_RXDAT_DATA(pSPI->RXDAT);
}
/**************************************************************************/
__attribute__ ((section(".bootloader_func")))
void Init_SST25_Bootloader()
{
	//SST25_Jedec_ID_Read_Bootloader();
	SST25_EnableWSR_Bootloader();
	SST25_WriteSR_Bootloader(0);               /* disable write protection */
	//SST25_WriteEnable_Bootloader();
}
/**************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_WriteSR_Bootloader(uint8_t data)
{

	TxBuf[0] = SST25_WRSR;
		TxBuf[1] = data;
		XfSetup.Length = 2;
		Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
		while(SST25_ReadBUSY_Bootloader());
//	putLineUART_Bootloader("WriteSR\r\n", 9);
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_EnableWSR_Bootloader()
{

	SST25_WriteEnable_Bootloader();
	TxBuf[0] =  SST25_EWSR;
	XfSetup.Length = 1;
	Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
	while(SST25_ReadBUSY_Bootloader());
	//putLineUART_Bootloader("EnableWSR\r\n", 11);
}
/****************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_WriteEnable_Bootloader()
{
	TxBuf[0] = SST25_WREN;
		XfSetup.Length = 1;
		Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
		while(SST25_ReadBUSY_Bootloader());
//	putLineUART_Bootloader("WriteEnable\r\n", 13);
}
/**************************************************************************/
__attribute__ ((section(".bootloader_func")))
 void SST25_EraseSector_Bootloader(uint32_t sector_no)
 {
	 uint32_t address = sector_no * 4096;

	  	SST25_WriteEnable_Bootloader();

	  	TxBuf[0] = SST25_SE;		/* erase 4K sector command	*/
	  	TxBuf[1] = WORD32_BYTE2(address);
	  	TxBuf[2] = WORD32_BYTE1(address);
	  	TxBuf[3] = WORD32_BYTE0(address);

	  	XfSetup.Length = 4;

	  	Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
	  	while(SST25_ReadBUSY_Bootloader());
 //	putLineUART_Bootloader("Erased\r\n", 8);
 }
/****************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_Jedec_ID_Read_Bootloader()
{

		TxBuf[0] = 0x9F;
		XfSetup.Length = 4;

		Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);

	/*	for(i = 0; i< 4; i++)
			LPC_UARTD_API->uart_put_char(uartHandle, RxBuf[i]);*/
}
/***************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_WriteByte_Bootloader(uint32_t address, uint8_t data)
{
	SST25_WriteEnable_Bootloader();

		TxBuf[0] = SST25_BP;
		TxBuf[1] = WORD32_BYTE2(address);
		TxBuf[2] = WORD32_BYTE1(address);
		TxBuf[3] = WORD32_BYTE0(address);
		TxBuf[4] = data;

		XfSetup.Length = 5;
		Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
}
/***********************************************************************************/
__attribute__ ((section(".bootloader_func")))
uint32_t SST25_WriteArray_Bootloader(char *array, uint16_t length, uint32_t u32_flashAddress)
{
	uint16_t i;

	for(i = 0; i< length; i++){
		if((u32_flashAddress & SST25_FLASH_SECTOR_MASK) == 0)
			SST25_EraseSector_Bootloader(u32_flashAddress/SST25_SECTOR_SIZE);

		SST25_WriteByte_Bootloader(u32_flashAddress, array[i]);
		while(SST25_ReadBUSY_Bootloader());
		u32_flashAddress++;
	}
	return u32_flashAddress;
}
/*******************************************************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_Read_Bootloader(uint32_t address, uint16_t length, char *out_buffer)
{
	uint16_t i;

	TxBuf[0] = SST25_READ;
	TxBuf[1] = WORD32_BYTE2(address);
	TxBuf[2] = WORD32_BYTE1(address);
	TxBuf[3] = WORD32_BYTE0(address);

	XfSetup.Length = length + 4;
//	XfSetup.pRx = spiRxBuffer;

	Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);

	for(i = 4; i < XfSetup.Length; i++)
		out_buffer[i - 4] = RxBuf[i];
}
/*********************************************************************************/
__attribute__ ((section(".bootloader_func")))
bool SST25_ReadBUSY_Bootloader()
{
	SST25_ReadSR_Bootloader();

		if(RxBuf[1] & 0x01)
			return TRUE;
		else
			return FALSE;
}
 /***********************************************************************************/
__attribute__ ((section(".bootloader_func")))
 void SST25_ReadSR_Bootloader()
 {
	 //PRINT_K("Reading status register...");
	 	TxBuf[0] = SST25_RDSR;
	 	XfSetup.Length = 2;
	 	Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
	  //	PRINT_K("Done\r\n");
 }
/*********************************************************************
 *
 * Function:    crcInit()
 *
 * Description: Populate the partial CRC lookup table.
 *
 * Notes:		This function must be rerun any time the CRC standard
 *				is changed.  If desired, it can be run "offline" and
 *				the table results stored in an embedded system's ROM.
 *
 * Returns:		None defined.
 *
 *********************************************************************/
#define WIDTH    (8 * sizeof(uint32_t))
#define TOPBIT   (1 << (WIDTH - 1))
#define POLYNOMIAL			0x04C11DB7
#define INITIAL_REMAINDER	0xFFFFFFFF
#define FINAL_XOR_VALUE		0xFFFFFFFF
#define REFLECT_DATA		TRUE
#define REFLECT_REMAINDER	TRUE

#define CHECK_VALUE			0xCBF43926
#if (REFLECT_DATA == TRUE)
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			((unsigned char) reflect((X), 8))
#else
#undef  REFLECT_DATA
#define REFLECT_DATA(X)			(X)
#endif

#if (REFLECT_REMAINDER == TRUE)
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	((uint32_t) reflect((X), WIDTH))
#else
#undef  REFLECT_REMAINDER
#define REFLECT_REMAINDER(X)	(X)
#endif
__attribute__ ((section(".bootloader_func")))
void crcInit(void)
{
    uint32_t	remainder;
	int32_t		dividend;
	uint8_t     bit;

    /*
     * Compute the remainder of each possible dividend.
     */
    for (dividend = 0; dividend < 256; ++dividend){
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (bit = 8; bit > 0; --bit){
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
                remainder = (remainder << 1) ^ POLYNOMIAL;
            else
                remainder = (remainder << 1);
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;

    }

}   /* crcInit() */


/*********************************************************************
 *
 * Function:    crcFast()
 *
 * Description: Compute the CRC of a given message.
 *
 * Notes:		crcInit() must be called first.
 *
 * Returns:		The CRC of the message.
 *
 *********************************************************************/
__attribute__ ((section(".bootloader_func")))
uint32_t crcFast(uint8_t const *message, int32_t nBytes)
{
    uint32_t  remainder = INITIAL_REMAINDER;
    uint8_t   data;
	int32_t   byte;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte){
        data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
  		remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);

}   /* crcFast() */
/*****************************************************/
__attribute__ ((section(".bootloader_func")))
static uint32_t reflect(uint32_t data, uint8_t nBits)
{
	uint32_t  reflection = 0x00000000;
	uint8_t  bit;

	/*
	 * Reflect the data about the center bit.
	 */
	for (bit = 0; bit < nBits; ++bit)
	{
		/*
		 * If the LSB bit is set, set the reflection of it.
		 */
		if (data & 0x01)
		{
			reflection |= (1 << ((nBits - 1) - bit));
		}

		data = (data >> 1);
	}

	return (reflection);

}	/* reflect() */
/*******************************************************/
#define CRC_CALC_BLOCK_SIZE       1024
__attribute__ ((section(".bootloader_func")))
uint32_t Calc_BlockCRC32(uint32_t u32_dataSize)
{
	uint32_t  remainder = INITIAL_REMAINDER;
	uint32_t  dataOffset;
	int32_t   byte;
	uint8_t   message[CRC_CALC_BLOCK_SIZE];
	uint8_t data;

	//putLineUART_Bootloader("\r\nCalculating checksum...\r\n", 40);
	dataOffset = FIRMWARE_ADDRESS;
	crcInit();

	while(u32_dataSize > 0) {

		SST25_Read_Bootloader(dataOffset, CRC_CALC_BLOCK_SIZE, (char *)message);
		u32_dataSize -= CRC_CALC_BLOCK_SIZE;
		dataOffset += CRC_CALC_BLOCK_SIZE;

		for (byte = 0; byte < CRC_CALC_BLOCK_SIZE; ++byte){
			  data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
			  remainder = crcTable[data] ^ (remainder << 8);
		}
	}
	return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
}
/*******************************************/
__attribute__ ((section(".bootloader_func")))
void SST25_ChipErase_Bootloader()
{
	//char spiRxBuffer[2];

	SST25_WriteEnable_Bootloader();


	TxBuf[0] = SST25_CE;       /* Erase chip command (C7h) */
	Chip_SPI_RWFrames_Blocking_Bootloader(LPC_SPI0, &XfSetup);
}
/************************************************************************************/
/* Prepare sector for write operation */
__attribute__ ((section(".bootloader_func")))
uint8_t IAP_PreSectorForReadWrite_Boot(uint32_t strSector, uint32_t endSector)
{
	uint32_t command[5], result[5];


	command[0] = IAP_PREWRRITE_CMD;
	command[1] = strSector;
	command[2] = endSector;
	iap_entry_bootloader(command, result);

	return result[0];

}
/****************************************************************************************/
/* Erase sector */
__attribute__ ((section(".bootloader_func")))
uint8_t IAP_EraseSector_Boot(uint32_t strSector, uint32_t endSector)
{
	uint32_t command[5], result[5];


	command[0] = IAP_ERSSECTOR_CMD;
	command[1] = strSector;
	command[2] = endSector;
	command[3] = ON_RESET_CORE_CLOCK / 1000;
	iap_entry_bootloader(command, result);


	return result[0];
}
/*******************************************************************************************/
/* Copy RAM to flash */
__attribute__ ((section(".bootloader_func")))
uint8_t IAP_CopyRamToFlash_Boot(uint32_t dstAdd, uint32_t *srcAdd, uint32_t byteswrt)
{
	uint32_t command[5], result[5];


	command[0] = IAP_WRISECTOR_CMD;
	command[1] = dstAdd;
	command[2] = (uint32_t) srcAdd;
	command[3] = byteswrt;
	command[4] = ON_RESET_CORE_CLOCK / 1000;
	iap_entry_bootloader(command, result);

	return result[0];
}
/********************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void iap_entry_bootloader(unsigned int cmd_param[], unsigned int status_result[])
{
	((IAP_ENTRY_T) IAP_ENTRY_LOCATION)(cmd_param, status_result);
}
/********************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void EnablePeriphClock_Bootloader(CHIP_SYSCTL_CLOCK_T clk)
{
	LPC_SYSCTL->SYSAHBCLKCTRL = (1 << clk) | (LPC_SYSCTL->SYSAHBCLKCTRL & ~SYSCTL_SYSAHBCLKCTRL_RESERVED);
}
/*********************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void DisablePeriphClock_Bootloader(CHIP_SYSCTL_CLOCK_T clk)
{
	LPC_SYSCTL->SYSAHBCLKCTRL &= ~((1 << clk) | SYSCTL_SYSAHBCLKCTRL_RESERVED);
}
/*******************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void Clock_SetUARTClockDiv_Bootloader(uint32_t div)
{
	LPC_SYSCTL->UARTCLKDIV = div;
}
/******************************************************************************/
/* assign a movable pin function to a physical pin */
#define PINASSIGN_IDX(movable)  (((movable) >> 4))
#define PINSHIFT(movable)       (((movable) & 0xF) << 3)

__attribute__ ((section(".bootloader_func")))
static void SWM_MovablePinAssign_Bootloader(CHIP_SWM_PIN_MOVABLE_T movable, uint8_t pin)
{
	uint32_t temp;
	int pinshift = PINSHIFT(movable), regIndex = PINASSIGN_IDX(movable);

	temp = LPC_SWM->PINASSIGN[regIndex] & (~(0xFF << pinshift));
	LPC_SWM->PINASSIGN[regIndex] = temp | (pin << pinshift);
}
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SYSCTL_PeriphReset_Bootloader(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	SYSCTL_AssertPeriphReset_Bootloader(periph);
	SYSCTL_DeassertPeriphReset_Bootloader(periph);
}
/******************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SYSCTL_AssertPeriphReset_Bootloader(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	LPC_SYSCTL->PRESETCTRL &= ~((1 << (uint32_t) periph) | SYSCTL_PRESETCTRL_RESERVED);
}
/*****************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SYSCTL_DeassertPeriphReset_Bootloader(CHIP_SYSCTL_PERIPH_RESET_T periph)
{
	LPC_SYSCTL->PRESETCTRL = (1 << (uint32_t) periph) | (LPC_SYSCTL->PRESETCTRL & ~SYSCTL_PRESETCTRL_RESERVED);
}
/*****************************************************************************/
__attribute__ ((section(".bootloader_func")))
static inline void SPI_Init_Bootloader()
{
	/* Enable SPI clock and reset IP */
	 EnablePeriphClock_Bootloader(SYSCTL_CLOCK_SPI0);
	 SYSCTL_PeriphReset_Bootloader(RESET_SPI0);
}
/*****************************************************************************/
__attribute__ ((section(".bootloader_func")))
void  *memset_boot(void *buffer, int c, int len)
{
  unsigned char *p = buffer;
  while(len > 0){
      *p = c;
      p++;
      len--;
    }
  return(buffer);
}
