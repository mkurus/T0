#include "board.h"
#include "timer.h"
#include "utils.h"

volatile TIMER_TICK_T timer_tick = 0;
/***************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) SysTick_Handler(void)  {
	MN_TICK_UPDATE;
}
/****************************************************************/
/* resets a timer                                               */
/****************************************************************/
void Set_Timer(PTIMER_INFO timer_ptr, TIMER_TICK_T num_ticks) {
	timer_ptr->timer_start = MN_GET_TICK;
	timer_ptr->timer_end = timer_ptr->timer_start + num_ticks;

	/* check for wrap */
	if (timer_ptr->timer_start <= timer_ptr->timer_end)
		timer_ptr->timer_wrap = FALSE;
	else {
		timer_ptr->timer_end = num_ticks
				- (0xffffffff - timer_ptr->timer_start + 1);
		timer_ptr->timer_wrap = TRUE;
	}
}
/********************************************************************/
/* returns 1 if a timer has expired, otherwise returns 0 */
/********************************************************************/
uint8_t mn_timer_expired(PTIMER_INFO timer_ptr) {
	TIMER_TICK_T curr_tick;

	curr_tick = MN_GET_TICK;

	if (timer_ptr->timer_wrap) {
		if ((curr_tick < timer_ptr->timer_start)
				&& (curr_tick > timer_ptr->timer_end))
			return TRUE;
	} else {
		if ((curr_tick > timer_ptr->timer_end)
				|| (curr_tick < timer_ptr->timer_start))
			return TRUE;
	}

	return FALSE;
}
/*********************************************************************/
TIMER_TICK_T mn_get_timer_tick(void) {
	volatile TIMER_TICK_T curr_tick;

	curr_tick = timer_tick;

	return (curr_tick);
}
/**********************************************************************/
/* waits for a given number of ticks. */
void mn_wait_ticks(TIMER_TICK_T num_ticks) {
	TIMER_INFO_T wait_timer;

	Set_Timer(&wait_timer, num_ticks);
	while (!mn_timer_expired(&wait_timer)){
//		onIdle();
	}
}
/************************************************************************/
void Init_WDT(uint16_t toutInSecs)
{
	uint32_t wdtFreq;

	/* Freq = 0.6Mhz, divided by 64. WDT_OSC should be 9.375khz */
	Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);

	/* Enable the power to the WDT */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);

	/* The WDT divides the input frequency into it by 4 */
	wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

	/* Initialize WWDT (also enables WWDT clock) */
	Chip_WWDT_Init(LPC_WWDT);

	/* Set watchdog feed time constant to approximately 1 minutes
	   Set watchdog window time to 3s */
	Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq * toutInSecs);
//	Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq * 3);

	/* Configure WWDT to reset on timeout */
	Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

	/* Start watchdog */
	Chip_WWDT_Start(LPC_WWDT);
}
