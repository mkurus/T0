/*
===============================================================================
 Name        : T0_Project.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include <stddef.h>
#include <string.h>
#include "board.h"
#include "timer.h"
#include "bsp.h"
#include "gsm.h"
#include "MMA8652.h"
#include "gps.h"
#include "messages.h"
#include "settings.h"
#include "status.h"
#include "sst25.h"
#include "spi.h"
#include "i2c.h"
#include "utils.h"

#endif
#endif

#include <cr_section_macros.h>
void Init_UART_PinMux();
void EnableBODReset();
void Configure_HardwarePinMuxes();
void Set_PinFunctionOutput(uint8_t pin_number, bool state);
extern bool volatile  b_i2ctransferComplete;
// TODO: insert other definitions and declarations here
uint8_t recv[10], send[10];
extern void _vStackTop(void);
extern float f_kmTempCounter;
int main(void) {
#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    	SystemCoreClockUpdate();
    	SysTick_Config(SystemCoreClock / TICKRATE_HZ1);
    	Configure_HardwarePinMuxes();

    	Init_GsmInfo();
    	Init_DEBUG_USART();
    	Trio_Init_GSM_UART();
    	Trio_ConfigureSimDetectInterrupt();
    	Trio_ConfigureIgnDetectInterrupt();
    	Init_EventInfo();

    	Set_PinFunctionOutput(GSM_RESET_PIN, FALSE);
    	Set_PinFunctionOutput(BLOCKAGE_PIN, FALSE);
    	Set_PinFunctionOutput(GPS_LED_PIN, FALSE);
    	Set_PinFunctionOutput(GSM_POWER_ON_OFF_PIN, FALSE);   /* turn off gsm module power */

    	//Trio_ConfigureGPSPowerTogglePin();
    	/*Board_LED_Init();*/
    	EnableBODReset();
#endif
#endif
    	//Trio_I2C_Init();
    	Trio_SPI_Init();
    	SST25_WriteSR(SST25_BP1);  /* enable flash protection */
    	Init_WDT(WATCHDOG_TIMEOUT_IN_SECS);

    	SST25_Jedec_ID_Read();
    //	SST25_ChipErase();


    	if(SST25_IsEmpty()) {
    		PRINT_K("Flash empty\n");
    		write_default_settings();
    	}
    	else
    		PRINT_K("Flash not empty\n");

    	Init_SPI_Cache();
    	Load_UserSettings();
   /* 	Load_BlockageSetting(); commented out for testing purpose */
    	Load_KmCounter();
    	Init_StatusInfo();

    	Trio_Init_GPS_UART();



    	ExecuteMainLoop();   /* never returns*/

    	NVIC_SystemReset();
}
/************************************************************************/
void Configure_HardwarePinMuxes()
{
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_Clock_SetUARTClockDiv(1);

	/* configure debug port UART */
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);

	/* configure GSM UART*/
	Chip_SWM_MovablePinAssign(SWM_U1_TXD_O, 18);
	Chip_SWM_MovablePinAssign(SWM_U1_RXD_I, 11);

	/* configure GPS UART*/
	Chip_SWM_MovablePinAssign(SWM_U2_TXD_O, 27);
	Chip_SWM_MovablePinAssign(SWM_U2_RXD_I, 16);

	/* Configure SPI pins*/
	Chip_SWM_MovablePinAssign(SWM_SPI0_SSEL0_IO, 26);
	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 24);
	Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, 25);
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, 15);

	/* Configure I2C pins*/
	Chip_SWM_MovablePinAssign(SWM_I2C1_SDA_IO, 6);
	Chip_SWM_MovablePinAssign(SWM_I2C1_SCL_IO, 7);

	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/************************************************************************/
 void Init_UART_PinMux(void)
{
	//Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
    Chip_Clock_SetUARTClockDiv(1);

	//Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1); //PIO0_0
	//Chip_SWM_DisableFixedPin(SWM_FIXED_SWCLK);

	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);

//	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}
 /*************************************************************************/
 void Set_PinFunctionOutput(uint8_t pin_number, bool state)
 {
	 Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	 Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, pin_number);
	 Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, pin_number, state);
	 Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
 }
 /*************************************************************************/
 void EnableBODReset()
 {
	 Chip_SYSCTL_SetBODLevels(SYSCTL_BODRSTLVL_2, SYSCTL_BODINTVAL_LVL0);
	 Chip_SYSCTL_EnableBODReset();
 }
