
#include "chip.h"
#include "bootloader.h"
#include <string.h>
#include "timer.h"

#if defined (__REDLIB__)
extern void __main(void);
#else
extern int main(void);
#endif


extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;
extern unsigned int __boot_section_table;
extern unsigned int __boot_section_table_end;
extern unsigned int __bss_section_table_end;
extern unsigned int __section_table_end;
extern unsigned int _bootloader_ram_size;

void Init_Sections();
void SystemInit();
//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}
/******************************************************************************/

__attribute__ ((used,section(".application_entry")))
void  __attribute__((long_call)) StartNormalOperation()
{
	Init_Sections();
	SystemInit();
	__main() ;
}
/*******************************************************************************/
void Init_Sections()
{
	extern unsigned int  _gps_variables_start;
	extern unsigned int _gps_variables_end;
	extern unsigned int _firmware_download_variables_start;
	extern unsigned int _firmware_download_variables_end;

	unsigned int LoadAddr, ExeAddr, SectionLen;
	unsigned int *SectionTableAddr;

	 SectionTableAddr = &__data_section_table;
     while (SectionTableAddr < &__data_section_table_end) {
    	 LoadAddr = *SectionTableAddr++;
    	 ExeAddr = *SectionTableAddr++;
    	 SectionLen = *SectionTableAddr++;
    	 data_init(LoadAddr, ExeAddr, SectionLen);
     }
     SectionTableAddr = &__bss_section_table;
     while (SectionTableAddr < &__bss_section_table_end) {
             ExeAddr = *SectionTableAddr++;
             SectionLen = *SectionTableAddr++;
             bss_init(ExeAddr, SectionLen);
     }


     memset(&_firmware_download_variables_start, 0, &_firmware_download_variables_end - &_firmware_download_variables_start);
     memset(&_gps_variables_start, 0, &_gps_variables_end - &_gps_variables_start);
}
/****************************************************************************/
/* The NMI handler */
//__attribute__ ((optimize("O0")))
__attribute__ ((used,section(".nmi_handler_jmp_address")))
void NMI_Handler_Jump_Vector() {
	NMI_IRQHandler();
}
/* The hard fault handler */
//__attribute__((optimize("O0")))
__attribute__ ((used,section(".hard_fault_handler_jmp_address")))
void HardFault_Handler_Jump_Vector() {
	HardFault_IRQHandler();
}
/* SVCall handler */
//__attribute__((optimize("O0")))
__attribute__ ((used,section(".svc_handler_jmp_address")))
void SVC_Handler_Jump_Vector() {
	SVC_IRQHandler();
}
/* The PendSV handler */
//__attribute__((optimize("O0")))
__attribute__ ((used,section(".pendsv_handler_jmp_address")))
void PendSV_Handler_Jump_Vector() {
	PendSV_IRQHandler();
}
/* The SysTick handler */
//__attribute__((optimize("O0")))
__attribute__ ((used,section(".systick_handler_jmp_address")))
void SysTick_Handler_Jump_Vector() {
	SysTick_Handler();
}
/*  SPI0 controller */
//__attribute__ ((optimize("O0")))
__attribute__ ((used,section(".spi0_isr_jmp_address")))
 void SPI0_IRQHandler_Jump_Vector() {
	SPI0_IRQHandler();
}
/* SPI1 controller */
__attribute__ ((used,section(".spi1_isr_jmp_address")))
void SPI1_IRQHandler_Jump_Vector() {
	SPI1_IRQHandler();
}
/* UART0 */
__attribute__ ((used,section(".uart0_isr_jmp_address")))
void UART0_IRQHandler_Jump_Vector() {
	UART0_IRQHandler();
}
/* UART1 */
__attribute__ ((used,section(".uart1_isr_jmp_address")))
void UART1_IRQHandler_Jump_Vector() {
	UART1_IRQHandler();
}
/* UART2 */
__attribute__ ((used,section(".uart2_isr_jmp_address")))
void UART2_IRQHandler_Jump_Vector() {
	UART2_IRQHandler();
}
/* I2C2 controller */
__attribute__ ((used,section(".i2c2_isr_jmp_address")))
void I2C2_IRQHandler_Jump_Vector() {
	I2C2_IRQHandler();
}
/* I2C0 controller */
__attribute__ ((used,section(".i2c0_isr_jmp_address")))
void I2C0_IRQHandler_Jump_Vector() {
	I2C0_IRQHandler();
}
/* I2C1 controller*/
__attribute__ ((used,section(".i2c1_isr_jmp_address")))
void I2C1_IRQHandler_Jump_Vector() {
	I2C1_IRQHandler();
}
/* I2C3 controller */
__attribute__ ((used,section(".i2c3_isr_jmp_address")))
void I2C3_IRQHandler_Jump_Vector() {
	I2C3_IRQHandler();
}
/*  Smart Counter Timer	*/
__attribute__ ((used,section(".sct_isr_jmp_address")))
void SCT_IRQHandler_Jump_Vector() {
	SCT_IRQHandler();
}
/* Multi-Rate Timer */
__attribute__ ((used,section(".mrt_isr_jmp_address")))
void MRT_IRQHandler_Jump_Vector() {
	MRT_IRQHandler();
}
/* Comparator */
__attribute__ ((used,section(".cmp_isr_jmp_address")))
void CMP_IRQHandler_Jump_Vector() {
	CMP_IRQHandler();
}
/* Watchdog	*/
__attribute__ ((used,section(".wdt_isr_jmp_address")))
void WDT_IRQHandler_Jump_Vector() {
	WDT_IRQHandler();
}
/* Brown Out Detect	*/
__attribute__ ((used,section(".bod_isr_jmp_address")))
void BOD_IRQHandler_Jump_Vector() {
	BOD_IRQHandler();
}
/* Flash Interrupt */
__attribute__ ((used,section(".flash_isr_jmp_address")))
void FLASH_IRQHandler_Jump_Vector() {
	FLASH_IRQHandler();
}
/* Wakeup timer */
__attribute__ ((used,section(".wkt_isr_jmp_address")))
void WKT_IRQHandler_Jump_Vector() {
	WKT_IRQHandler();
}
/* ADC sequence A completion */
__attribute__ ((used,section(".adc_seqa_isr_jmp_address")))
void ADC_SEQA_IRQHandler_Jump_Vector(){
	ADC_SEQA_IRQHandler();
}
/* ADC sequence B completion */
__attribute__ ((used,section(".adc_seqb_isr_jmp_address")))
void ADC_SEQB_IRQHandler_Jump_Vector() {
	ADC_SEQB_IRQHandler();
}
/* ADC threshold compare */
__attribute__ ((used,section(".adc_thcmp_isr_jmp_address")))
void ADC_THCMP_IRQHandler_Jump_Vector() {
	ADC_THCMP_IRQHandler();
}
/* ADC overrun */
__attribute__ ((used,section(".adc_ovr_isr_jmp_address")))
void ADC_OVR_IRQHandler_Jump_Vector() {
	ADC_OVR_IRQHandler();
}
/* DMA */
__attribute__ ((used,section(".dma_isr_jmp_address")))
void DMA_IRQHandler_Jump_Vector() {
	DMA_IRQHandler();
}
/* PIO INT0	*/
__attribute__ ((used,section(".pin_int0_isr_jmp_address")))
void PIN_INT0_IRQHandler_Jump_Vector() {
	PIN_INT0_IRQHandler();
}
/* PIO INT1	*/
__attribute__ ((used,section(".pin_int1_isr_jmp_address")))
void PIN_INT1_IRQHandler_Jump_Vector() {
	PIN_INT1_IRQHandler();
}
/* PIO INT2	*/
__attribute__ ((used,section(".pin_int2_isr_jmp_address")))
void PIN_INT2_IRQHandler_Jump_Vector() {
	PIN_INT2_IRQHandler();
}
/* PIO INT3	*/
__attribute__ ((used,section(".pin_int3_isr_jmp_address")))
void PIN_INT3_IRQHandler_Jump_Vector() {
	PIN_INT3_IRQHandler();
}
/* PIO INT4	*/
__attribute__ ((used,section(".pin_int4_isr_jmp_address")))
void PIN_INT4_IRQHandler_Jump_Vector() {
	PIN_INT4_IRQHandler();
}
/* PIO INT5	*/
__attribute__ ((used,section(".pin_int5_isr_jmp_address")))
void PIN_INT5_IRQHandler_Jump_Vector() {
	PIN_INT5_IRQHandler();
}
/* PIO INT6	*/
__attribute__ ((used,section(".pin_int6_isr_jmp_address")))
void PIN_INT6_IRQHandler_Jump_Vector() {
	PIN_INT6_IRQHandler();
}
/* PIO INT7	*/
__attribute__ ((used,section(".pin_int7_isr_jmp_address")))
void PIN_INT7_IRQHandler_Jump_Vector() {
	PIN_INT7_IRQHandler();
}
/***************************************************************/
__attribute__((optimize("-O0")))
void IntDefaultHandler(void)  {
	while (1);
}
/*****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) NMI_IRQHandler()
{
	IntDefaultHandler();
}
/*****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) HardFault_IRQHandler()
{
	IntDefaultHandler();
}
/*****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) SVC_IRQHandler()
{
	IntDefaultHandler();
}
/*****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PendSV_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) I2C1_IRQHandler()
{
	I2C1_InterruptHandler();
}
/****************************************************************/
__attribute__((short_call))
void  __attribute__ ((noinline)) I2C2_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline))  I2C3_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) SCT_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) MRT_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) CMP_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) WDT_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) BOD_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) FLASH_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) WKT_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) ADC_SEQA_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) ADC_SEQB_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) ADC_THCMP_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) ADC_OVR_IRQHandler()
{
	 IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) DMA_IRQHandler()
{
	 IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PIN_INT3_IRQHandler()
{
	 IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PIN_INT4_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PIN_INT5_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PIN_INT6_IRQHandler()
{
	IntDefaultHandler();
}
/****************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PIN_INT7_IRQHandler()
{
	IntDefaultHandler();
}
