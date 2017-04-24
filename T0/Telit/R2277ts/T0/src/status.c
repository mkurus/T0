/*
 * status.c
 *
 *  Created on: 3 Ağu 2015
 *      Author: admin
 */

#include "board.h"
#include "timer.h"
#include "messages.h"
#include "settings.h"
#include "bsp.h"
#include "gsm.h"
#include "spi.h"
#include "utils.h"
#include "status.h"
#include "gps.h"
#include "bootloader.h"
#include "settings.h"
#include <stdlib.h>
#include <string.h>

typedef enum POWER_STATE_T{
	POWER_REMOVED_STATE,
	POWER_APPLIED_STATE,
}POWER_STATES;

typedef enum SIM_STATE_T{
	SIM_REMOVED_STATE,
	SIM_INSERTED_STATE,
}SIM_STATES;

typedef enum IGN_STATE_T{
	NOT_IGNITED_STATE,
	IGNITED_STATE
}IGN_STATES;

typedef enum SPEED_STATE_T{
	BELOW_SPEED_LIMIT_STATE,
	ABOVE_SPEED_LIMIT_STATE,
	WAIT_SPEED_ABOVE_LIMIT_TIMEOUT_STATE
}SPEED_STATES;

typedef enum STOP_STATES_T{
	STOPPED_STATE,
	NOT_STOPPED_STATE,
    WAITING_STOP_TIMEOUT_STATE
}STOP_STATES;

typedef struct SIM_STATUS_INFO_T
{
	bool b_simInserted;
	SIM_STATES simState;
	TIMER_INFO_T sim_debounce_timer;
}SIM_STATUS_INFO;

typedef struct IGNITION_INFO_T
{
	bool b_ignited;
	bool b_intIgnited;
	IGN_STATES ignState;
	TIMER_INFO_T ign_debounce_timer;
}IGNITION_INFO;

typedef struct POWER_STATUS_INFO_T
{
	bool b_powered;
	POWER_STATES powerState;
	TIMER_INFO_T power_debounce_timer;
}POWER_STATUS_INFO;

typedef struct SPEED_LIMIT_INFO_T
{
	TIMER_INFO_T speed_alarm_timer;
	TIMER_INFO_T speed_secs_counter_timer;
	TIMER_TICK_T u32_totalSpeedViolationTime;
	SPEED_STATES state;
}SPEED_LIMIT_INFO;

typedef struct IDLE_INFO_T
{
	TIMER_INFO_T idle_alarm_timer;
	TIMER_INFO_T idle_secs_counter_timer;
	TIMER_TICK_T u32_totalIdleTime;
	TIMER_TICK_T u16_idleTimeBetweenMessage;
	IDLE_STATES_T state;
}IDLE_INFO;

typedef struct STOP_INFO_T
{
	TIMER_INFO_T stop_alarm_timer;
	TIMER_INFO_T stop_secs_counter_timer;
	TIMER_TICK_T u32_totalStopTime;
	STOP_STATES  state;
}STOP_INFO;

#define TRIP_START_INFO_BUFFER_SIZE        64

typedef struct TRIP_INFO{
	char tripStartInfoBuffer[TRIP_START_INFO_BUFFER_SIZE];
	uint32_t u32_tripStartKm;
	uint32_t u32_tripEndKm;
	uint32_t u32_totalTripIdleTime;
	GPS_TRIP_STATUS_T gpsStatus;
}TRIP_INFO_T;

TRIP_INFO_T trip_info;
SIM_STATUS_INFO sim_status_info;
IGNITION_INFO ignition_info;
POWER_STATUS_INFO power_status_info;
SPEED_LIMIT_INFO speed_limit_info;
STOP_INFO stop_info;
IDLE_INFO idle_info;
STATUS_INFO_T status_info;
EVENT_INFO_T event_info;
BLOCKAGE_INFO_T blockage_info;

void Init_SIMInfo();
void Init_IgnInfo();
void Init_PowerInfo();
void Init_IdleInfo();
void Init_StopInfo();
void Init_SpeedLimitInfo();

//#ifdef 	T0_TELIT_GL865
#define SUPPLY_VOLTAGE_LOW_THRESHOLD       4000      /* /10 in mV */
//#elif   defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
//#define SUPPLY_VOLTAGE_LOW_THRESHOLD       4000      /* in mV */
//#endif

#define SIM_DEBOUNCE_TIME                  10
#define IGN_DEBOUNCE_TIME                  (3 * SECOND)
#define POWER_DEBOUNCE_TIME                10

#define MIN_NUMBER_OF_SATS_FOR_BLOCKAGE    5
void IncreaseIdleTimeCounters();

static void ProcessSIMStatus();
static void ProcessIgnStatus(NMEA_GPRMC_STATUS_T gpsStatus);
static void ProcessBlockage(RMC_MESSAGE_T *rmc_info, GGA_MESSAGE_T *gga_info);
static void ProcessIdleStatus(uint16_t u16_speedKm, uint32_t u32_maxIdleTime);
static void ProcessSpeedLimitStatus(uint16_t u16_speedKm);
static void ProcessStopTimeLimitStatus();
static void ProcessExtPowerStatus();

static bool b_currentIgnStatus;
static bool b_prevIgnStatus;
static bool b_currentPwrStatus;
static bool b_prevPwrStatus;
/***********************************************************************************/
void Trio_StatusTask()
{
	FLASH_SETTINGS_T user_settings;
	RMC_MESSAGE_T rmc_info;
	GGA_MESSAGE_T gga_info;
	GSA_MESSAGE_T gsa_info;

	Get_UserSettings(&user_settings);

	Get_GGAInfo(&gga_info);
	Get_GSAInfo(&gsa_info);
	Get_RMCInfo(&rmc_info);


	ProcessBlockage(&rmc_info, &gga_info);
	ProcessSIMStatus();
	ProcessIgnStatus(rmc_info.status);

	if(user_settings.u32_maxIdleTime > 0 && ignition_info.b_ignited &&
		(gsa_info.mode2 == NMEA_GPGSA_MODE2_3D) &&
		(gga_info.position_fix))
		ProcessIdleStatus(rmc_info.speed, user_settings.u32_maxIdleTime);

	if((user_settings.u16_speedLimitViolationDuration > 0) &&
	   (gsa_info.mode2 == NMEA_GPGSA_MODE2_3D) &&
	   (gga_info.position_fix))
	   ProcessSpeedLimitStatus(rmc_info.speed*1852 /1000);

	ProcessStopTimeLimitStatus();
	ProcessExtPowerStatus();
}
/**********************************************************************************/
/**
 *Handle interrupt from PININT channel 0 (sim detection interrupt)
 */
__attribute__((short_call))
void __attribute__ ((noinline)) PININT0_IRQHandler(void)
{
	uint32_t u32_riseStates;
	uint32_t u32_fallStates;

	u32_riseStates = Chip_PININT_GetRiseStates(LPC_PININT);
	u32_fallStates = Chip_PININT_GetFallStates(LPC_PININT);

	if(u32_riseStates & PININTCH0){
		Chip_PININT_ClearRiseStates(LPC_PININT, PININTCH0);
	//	sim_status_info.b_simInserted  = FALSE;
	}
	else if(u32_fallStates & PININTCH0){
		Chip_PININT_ClearFallStates(LPC_PININT, PININTCH0);
	//	sim_status_info.b_simInserted = TRUE;
	}
	sim_status_info.b_simInserted = !Get_SimDetectPinStatus();
	Set_Timer(&(sim_status_info.sim_debounce_timer), SIM_DEBOUNCE_TIME);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
}

/**********************************************************************************/
void Trio_ConfigureSimDetectInterrupt()
{
	/* Initialize pin interrupt */
	Chip_SYSCTL_SetPinInterrupt(0, SIM_DETECT_PIN);							/* Set pin interrupt 0 to SIMDETECT pin*/
	Chip_PININT_Init(LPC_PININT);											/* initialize pin interrupt module */
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);				        /* set pin interrupt channel 0 to be edge sensitive */
	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH0);						/* interrupt on rising edge */
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH0);	                    /* interrupt on low level */
	NVIC_EnableIRQ(PININT0_IRQn);											/* enable pin interrupt 0 */
}

/***********************************************************************************/
void ProcessSIMStatus()
{
	switch(sim_status_info.simState)
	{
		case SIM_REMOVED_STATE:
		if(sim_status_info.b_simInserted){
			if(mn_timer_expired(&(sim_status_info.sim_debounce_timer))){
				PRINT_K("\n@@@@.....SIM_INSERTED...@@@@@\n");
				sim_status_info.simState = SIM_INSERTED_STATE;
			}
		}
		break;

		case SIM_INSERTED_STATE:
		if(!sim_status_info.b_simInserted){
			if(mn_timer_expired(&(sim_status_info.sim_debounce_timer))){
				PRINT_K("\n@@@@......SIM_REMOVED.....@@@@@\n");
				sim_status_info.simState = SIM_REMOVED_STATE;
				event_info.event_sim_card_removed = TRUE;
			}
		}
		break;
	}
}
/************************************************************/
/* Interrupt service routine for ignition detection         */
/************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) PININT1_IRQHandler(void)
{

	uint32_t u32_riseStates;
	uint32_t u32_fallStates;
	IntDefaultHandler();
  return;
	u32_riseStates = Chip_PININT_GetRiseStates(LPC_PININT);
	u32_fallStates = Chip_PININT_GetFallStates(LPC_PININT);

	if(u32_riseStates & PININTCH1){
		Chip_PININT_ClearRiseStates(LPC_PININT, PININTCH1);
		ignition_info.b_intIgnited = FALSE;
	//	ignition_info.b_ignited = FALSE;
	}
	else if(u32_fallStates & PININTCH1){
		Chip_PININT_ClearFallStates(LPC_PININT, PININTCH1);
		ignition_info.b_intIgnited = TRUE;
	//	ignition_info.b_ignited = TRUE;
	}

	Set_Timer(&(ignition_info.ign_debounce_timer), IGN_DEBOUNCE_TIME);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
}
/****************************************************************/
void Trio_ConfigureIgnDetectInterrupt()
{
	return;
	/* Initialize pin interrupt */
	Chip_SYSCTL_SetPinInterrupt(1, IGN_DETECT_PIN);							/* Set pin interrupt 0 to IGNDETECT pin*/
	Chip_PININT_Init(LPC_PININT);											/* initialize pin interrupt module */
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);				        /* set pin interrupt channel 0 to be edge sensitive */
	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH1);						/* interrupt on high level */
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH1);	                    /* interrupt on low level */
	NVIC_EnableIRQ(PININT1_IRQn);											/* enable pin interrupt 0 */
}
/****************************************************************/
void ProcessIgnStatus2(NMEA_GPRMC_STATUS_T status)
{
	GPS_POSITION_DATA_T current_position;

	b_currentIgnStatus = Get_IgnitionPinStatus();

	if(b_currentIgnStatus != b_prevIgnStatus){
		Set_Timer(&ignition_info.ign_debounce_timer, IGN_DEBOUNCE_TIME);
		b_prevIgnStatus =  b_currentIgnStatus;
		ignition_info.b_intIgnited = b_currentIgnStatus;
	}

	switch(ignition_info.ignState)
	{
		case NOT_IGNITED_STATE:
		if(ignition_info.b_intIgnited){
			if(mn_timer_expired(&(ignition_info.ign_debounce_timer))){
				ignition_info.b_ignited = TRUE;
				ReloadPeriodicDataSendTimer();
				PRINT_K("\nIGNITED\n");
				ReloadLogTimer();
				if(status == NMEA_GPRMC_VALID)
					Init_TripInfo(VALID_GPS_ON_IGNITION);
				else
					Init_TripInfo(INVALID_GPS_ON_IGNITION);
				Init_IdleInfo();
				ignition_info.ignState = IGNITED_STATE;
				event_info.event_ignition_changed_off_to_on = TRUE;
			}
		}
		break;

		case IGNITED_STATE:
		if(!ignition_info.b_intIgnited){
			if(mn_timer_expired(&(ignition_info.ign_debounce_timer))){
				ignition_info.b_ignited = FALSE;
				PRINT_K("\nNOT_IGNITED\n");
				ReloadPeriodicDataSendTimer();
				ReloadLogTimer();
				Get_PositionInfo(&current_position);
				trip_info.u32_tripEndKm = current_position.distance;
				ignition_info.ignState = NOT_IGNITED_STATE;
				event_info.event_ignition_changed_on_to_off = TRUE;
			}
		}
		break;
	}
}
/****************************************************************/
void ProcessIgnStatus(NMEA_GPRMC_STATUS_T status)
{
	GPS_POSITION_DATA_T current_position;

	b_currentIgnStatus = Get_IgnitionPinStatus();

	if(b_currentIgnStatus != b_prevIgnStatus){
		b_prevIgnStatus =  b_currentIgnStatus;
		if(b_currentIgnStatus){
			ignition_info.b_ignited = TRUE;
			ReloadPeriodicDataSendTimer();
			PRINT_K("\nIGNITED\n");
			ReloadLogTimer();
			if(status == NMEA_GPRMC_VALID)
				Init_TripInfo(VALID_GPS_ON_IGNITION);
			else
				Init_TripInfo(INVALID_GPS_ON_IGNITION);
			Init_IdleInfo();
			event_info.event_ignition_changed_off_to_on = TRUE;
		}
		else{
			ignition_info.b_ignited = FALSE;
			PRINT_K("\nNOT_IGNITED\n");
			ReloadPeriodicDataSendTimer();
			ReloadLogTimer();
			Get_PositionInfo(&current_position);
			trip_info.u32_tripEndKm = current_position.distance;
			ignition_info.ignState = NOT_IGNITED_STATE;
			event_info.event_ignition_changed_on_to_off = TRUE;
		}
	}
}
/**********************************************************************/
GPS_TRIP_STATUS_T Get_TripGpsStatus()
{
	return trip_info.gpsStatus;
}
/***********************************************************************/
bool Get_IgnitionStatus()
{
	return ignition_info.b_ignited;
}
/***********************************************************************/
char * Get_TripStartInfoBufferPtr()
{
	return trip_info.tripStartInfoBuffer;
}
/***********************************************************************/
uint32_t Get_TripIdleTime()
{
	return trip_info.u32_totalTripIdleTime;
}
/***********************************************************************/
uint32_t Get_TripDistance()
{
	return trip_info.u32_tripEndKm - trip_info.u32_tripStartKm;
}
/***********************************************************************/
bool Get_EventStatus()
{
	uint32_t *u32_alarmStatus;

	u32_alarmStatus = (uint32_t *)&event_info;
	if(*u32_alarmStatus != 0)
		return TRUE;
	else
		return FALSE;
}
/************************************************************************/
void ProcessIdleStatus(uint16_t u16_speedKm, uint32_t u32_maxIdleTime)
{
	switch(idle_info.state)
		{
			case IDLE_STATE:
			if(u16_speedKm > IDLE_SPEED_THRESHOLD){
				idle_info.state = NOT_IDLE_STATE;
				PRINT_K("Out of IDLE state\n");
				event_info.event_idle_time_finished = SET;
			}
			else{
				if(mn_timer_expired(&idle_info.idle_secs_counter_timer)){
					IncreaseIdleTimeCounters();
					Set_Timer(&idle_info.idle_secs_counter_timer, SECOND);
				}
			/*	if(mn_timer_expired(&(idle_info.idle_alarm_timer))){
					event_info.event_idle_time_started = SET;
					Set_Timer(&(idle_info.idle_alarm_timer), u32_maxIdleTime *100);
					PRINT_K("Idle timer started\n");
				}*/
			}
			break;

			case WAITING_IDLE_TIMEOUT_STATE:
			if(u16_speedKm > IDLE_SPEED_THRESHOLD){
				idle_info.state = NOT_IDLE_STATE;
				ResetTotalIdleTime();
				PRINT_K("Timeout beklerken hareket edildi\n");
			}
			else{
				if(mn_timer_expired(&(idle_info.idle_alarm_timer))){
					event_info.event_idle_time_started = SET;
					PRINT_K("Idle Timeout alarmi olustu\n");
					idle_info.state = IDLE_STATE;
				}
				else if(mn_timer_expired(&idle_info.idle_secs_counter_timer)){
						IncreaseIdleTimeCounters();
						Set_Timer(&idle_info.idle_secs_counter_timer, SECOND);
				}
			}
			break;

			case NOT_IDLE_STATE:
			if(u16_speedKm <= IDLE_SPEED_THRESHOLD){
				Set_Timer(&idle_info.idle_secs_counter_timer, SECOND);
				Set_Timer(&(idle_info.idle_alarm_timer), u32_maxIdleTime *100);
				idle_info.state = WAITING_IDLE_TIMEOUT_STATE;
				PRINT_K("Waiting Idle Timeout\n");
			}
			break;
		}
}
/*************************************************************************/
void IncreaseIdleTimeCounters()
{
	idle_info.u32_totalIdleTime++;
	trip_info.u32_totalTripIdleTime++;
	idle_info.u16_idleTimeBetweenMessage++;
}
/*************************************************************************/
void ResetInterMessageIdleTime()
{
	idle_info.u16_idleTimeBetweenMessage = 0;
}

/*************************************************************************/
void ProcessSpeedLimitStatus(uint16_t u16_speedKm)
{

  // DumpStackPointer();
	uint16_t u16_speedLimit, timeout;

	u16_speedLimit =  Get_MaxSpeedLimit();
	timeout =  Get_MaxSpeedViolationDuration();

	switch(speed_limit_info.state)
	{
		case BELOW_SPEED_LIMIT_STATE:
		if(u16_speedKm > u16_speedLimit){
			PRINT_K("Hiz limiti asildi\r\n");
			Set_Timer(&(speed_limit_info.speed_alarm_timer), timeout * SECOND);
			Set_Timer(&speed_limit_info.speed_secs_counter_timer, SECOND);
			speed_limit_info.state = WAIT_SPEED_ABOVE_LIMIT_TIMEOUT_STATE;
		}
		break;

		case WAIT_SPEED_ABOVE_LIMIT_TIMEOUT_STATE:
		if(u16_speedKm <= u16_speedLimit){
			speed_limit_info.state = BELOW_SPEED_LIMIT_STATE;
			ResetSpeedViolationTime();
		/*	PRINT_K("\r\nTimeout beklerken hiz limitini altina inildi\r\n");*/
		}
		else if(mn_timer_expired(&speed_limit_info.speed_alarm_timer)) {
			event_info.event_speed_limit_violation_started = SET;
			speed_limit_info.state = ABOVE_SPEED_LIMIT_STATE;
			PRINT_K("Hiz limiti timeout\r\n");
		}
		if(mn_timer_expired(&speed_limit_info.speed_secs_counter_timer)){
			speed_limit_info.u32_totalSpeedViolationTime++;
			Set_Timer(&speed_limit_info.speed_secs_counter_timer, SECOND);
		}
		break;

		case ABOVE_SPEED_LIMIT_STATE:
		if(u16_speedKm <= u16_speedLimit){
		/*	PRINT_K("\Hiz limitinin altina inildi\r\n");*/
			speed_limit_info.state = BELOW_SPEED_LIMIT_STATE;
			event_info.event_speed_limit_violation_finished = SET;
		}
		else{
			if(mn_timer_expired(&speed_limit_info.speed_secs_counter_timer)){
				speed_limit_info.u32_totalSpeedViolationTime++;
				Set_Timer(&speed_limit_info.speed_secs_counter_timer, SECOND);
			}
		}
		break;

	}
}
/************************************************************************/
void ProcessStopTimeLimitStatus()
{
	uint32_t u32_stopTimeLimit;

	u32_stopTimeLimit = Get_MaxStopTime();

	if(u32_stopTimeLimit <= 0)
		return;

	switch(stop_info.state){
		case STOPPED_STATE:
		if(Get_IgnitionStatus()){
			stop_info.state = NOT_STOPPED_STATE;
			//PRINT_K("\r\nOut of Stop State\r\n");
			event_info.event_stop_time_finished = SET;
		}
		else if(mn_timer_expired(&stop_info.stop_secs_counter_timer)){
				stop_info.u32_totalStopTime++;
				Set_Timer(&stop_info.stop_secs_counter_timer, SECOND);
		}
		/*	if(mn_timer_expired(&stop_info.stop_alarm_timer)) {
					event_info.event_stop_time_started = SET;
					Set_Timer(&stop_info.stop_alarm_timer, user_settings.u16_maxStopTime * 100);
					PRINT_K("\r\nStop Timeout alarmi olustu\r\n");
			}*/
		break;

		case WAITING_STOP_TIMEOUT_STATE:
		if(Get_IgnitionStatus()){
			stop_info.state = NOT_STOPPED_STATE;
			ResetStopTime();
			//PRINT_K("\r\nTimeout beklerken kontak acildi\r\n");
		}
		else{
			if(mn_timer_expired(&stop_info.stop_alarm_timer)) {
				event_info.event_stop_time_started = SET;
				stop_info.state = STOPPED_STATE;
			}
			else if(mn_timer_expired(&stop_info.stop_secs_counter_timer)){
				stop_info.u32_totalStopTime++;
				Set_Timer(&stop_info.stop_secs_counter_timer, SECOND);
			}
		}
		break;

		case NOT_STOPPED_STATE:
		if(!Get_IgnitionStatus()){
			Set_Timer(&stop_info.stop_alarm_timer, u32_stopTimeLimit * SECOND);
			Set_Timer(&stop_info.stop_secs_counter_timer,  SECOND);
			stop_info.state = WAITING_STOP_TIMEOUT_STATE;
		//	PRINT_K("\r\nWaiting Stop Timeout\r\n");
		}
		break;


	}
}
/****************************************************************/
void Trio_ConfigureExtPowerDetectInterrupt()
{
	/* Initialize pin interrupt */
	//Chip_SYSCTL_SetPinInterrupt(2, POWER_DETECT_PIN);						/* Set pin interrupt 2 for ext. power detection*/
	//Chip_PININT_Init(LPC_PININT);											/* initialize pin interrupt module */
//	Chip_PININT_SetPinModeLevel(LPC_PININT, PININTCH2);				        /* set pin interrupt channel 2 to be edge sensitive */
//	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH2);						/* enable level interrupt*/
//	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH2);	                    /* interrupt on high level */
//	Chip_PININT_DisableIntLow(LPC_PININT, PININTCH2);                       /* interrupt on low level*/
//	NVIC_EnableIRQ(PININT2_IRQn);											/* enable pin interrupt 2 */
}
/**********************************************************************************/
/**
 *Handle interrupt from PININT channel 2 (External Power Status interrupt)

 Pin interrupt registers for edge-and level-sensitive pins

 Name	   Edge-sensitive function	                      Level-sensitive function
 IENR	   Enables rising-edge interrupts.	              Enables level interrupts.
 SIENR	   Write to enable rising-edge interrupts.	      Write to enable level interrupts.
 CIENR	   Write to disable rising-edge interrupts.	      Write to disable level interrupts.
 IENF	   Enables falling-edge interrupts.	              Selects active level.
 SIENF	   Write to enable falling-edge interrupts.	      Write to select high-active.
 CIENF	   Write to disable falling-edge interrupts.	  Write to select low-active.
 */
__attribute__((short_call))
void __attribute__ ((noinline)) PININT2_IRQHandler(void)
{
	IntDefaultHandler();
/**	power_status_info.b_powered  = !power_status_info.b_powered;
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
	Set_Timer(&(power_status_info.power_debounce_timer), POWER_DEBOUNCE_TIME);*/
}
/************************************************************************/
void ProcessExtPowerStatus()
{
	b_currentPwrStatus = Get_PowerDetectPinStatus();

	if(b_currentPwrStatus != b_prevPwrStatus){
		Set_Timer(&power_status_info.power_debounce_timer, POWER_DEBOUNCE_TIME);
		b_prevPwrStatus =  b_currentPwrStatus;
		power_status_info.b_powered = b_currentPwrStatus;
	}
	switch(power_status_info.powerState){
		case POWER_REMOVED_STATE:
		if(power_status_info.b_powered){
			if(mn_timer_expired(&power_status_info.power_debounce_timer))
			{//	PRINT_K("POWER_APPLIED\n");
				if(GetBatteryVoltage() > SUPPLY_VOLTAGE_LOW_THRESHOLD){
					PRINT_K("POWER_APPLIED\n");
					power_status_info.powerState = POWER_APPLIED_STATE;
					event_info.event_power_status_changed = SET;
				}
			}
		}
		break;

		case POWER_APPLIED_STATE:
		if(!power_status_info.b_powered){
			if(mn_timer_expired(&power_status_info.power_debounce_timer)){
				//PRINT_K("POWER_REMOVED\n");
				if(GetBatteryVoltage() <= SUPPLY_VOLTAGE_LOW_THRESHOLD){
					PRINT_K("POWER_REMOVED\n");
					power_status_info.powerState = POWER_REMOVED_STATE;
					event_info.event_power_status_changed = SET;
				}
			}
		}
		break;
	}
}
/************************************************************************/
static void ProcessBlockage(RMC_MESSAGE_T *rmc_info, GGA_MESSAGE_T *gga_info)
{

	if(blockage_info.blockageStatus ==  BLOCKAGE_REQUESTED){
		if(!Get_IgnitionStatus() && (rmc_info->status == NMEA_GPRMC_VALID) &&
				(rmc_info->speed *1852/1000 == 0) &&  (gga_info->position_fix) &&
				(gga_info->satellites >= MIN_NUMBER_OF_SATS_FOR_BLOCKAGE) ){
			SetBlockage(TRUE);
			Set_BlockageStatus(BLOCKAGE_ACTIVATED);
			PRINT_K("BLOCKAGE ACTIVATED\n");
		}
	}
	else if(blockage_info.blockageStatus ==  BLOCKAGE_REMOVE_REQUESTED){
		SetBlockage(FALSE);
		Set_BlockageStatus(BLOCKAGE_REMOVED);
		PRINT_K("BLOCKAGE REMOVED\n");
	}
}
/************************************************************************/
void Get_BlockageInfo(BLOCKAGE_INFO_T *blockage)
{
	memcpy(blockage, &blockage_info, sizeof(BLOCKAGE_INFO_T));
}
/************************************************************************/
void Set_BlockageSource(COMMAND_SOURCE_T source)
{
	blockage_info.cmdSource = source;
}
/************************************************************************/
void Set_BlockageTransID(char *transactionID)
{
	memcpy(blockage_info.transactionID, transactionID, sizeof(blockage_info.transactionID));
}
/************************************************************************/
void Get_BlockageTransID(char *transactionID)
{
	memcpy(transactionID, blockage_info.transactionID, sizeof(blockage_info.transactionID));
}
/************************************************************************/
void SetBlockage(bool blockageStatus)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, BLOCKAGE_PIN, blockageStatus);
}
/************************************************************************/
void Get_EventInfo(EVENT_INFO_T *event_info_t)
{
	*event_info_t = event_info;
}
/************************************************************************/
void Set_BlockageStatus(BLOCKAGE_STATUS_T new_status)
{
	blockage_info.blockageStatus = new_status;
}
/*************************************************************************/
void Get_StatusInfo(STATUS_INFO_T *t_status)
{
	*t_status = status_info;
}
/*************************************************************************/
void Init_StatusInfo()
{
	status_info.power_status = STATUS_EXT_POWERED;
	Init_SIMInfo();
	Init_IgnInfo();
	Init_PowerInfo();
	Init_IdleInfo();
	Init_StopInfo();
	Init_SpeedLimitInfo();
//	memset(&event_info, 0, sizeof(event_info));

}
/***************************************************************************/
void Init_TripInfo(GPS_TRIP_STATUS_T status)
{
	GPS_POSITION_DATA_T current_position;

	Get_PositionInfo(&current_position);

	memset(trip_info.tripStartInfoBuffer, 0, TRIP_START_INFO_BUFFER_SIZE);
	Trio_StartTripInfoMessage(trip_info.tripStartInfoBuffer);

	trip_info.u32_totalTripIdleTime = 0;
	trip_info.u32_tripStartKm = current_position.distance;
	trip_info.gpsStatus = status;
}
/***************************************************************************/
void Init_SIMInfo()
{
	if(!Get_SimDetectPinStatus()){
		sim_status_info.simState = SIM_INSERTED_STATE;
		sim_status_info.b_simInserted = TRUE;
	}
	else{
		sim_status_info.simState = SIM_REMOVED_STATE;
		sim_status_info.b_simInserted = FALSE;
	}
}
/***************************************************************************/
void Init_IgnInfo()
{
	if(Get_IgnitionPinStatus()){
		b_currentIgnStatus = b_prevIgnStatus = TRUE;
		ignition_info.ignState = IGNITED_STATE;
		ignition_info.b_intIgnited = ignition_info.b_ignited = TRUE;

	}
	else{
		b_currentIgnStatus = b_prevIgnStatus = FALSE;
		ignition_info.ignState = NOT_IGNITED_STATE;
		ignition_info.b_intIgnited = ignition_info.b_ignited = FALSE;
	}
}
/****************************************************************************/
void Init_PowerInfo()
{
	if(Get_PowerDetectPinStatus()){
		power_status_info.powerState = POWER_APPLIED_STATE;
		power_status_info.b_powered = TRUE;
		b_currentPwrStatus = b_prevPwrStatus = TRUE;
	}
	else{
		power_status_info.powerState = POWER_REMOVED_STATE;
		power_status_info.b_powered = FALSE;
		b_currentPwrStatus = b_prevPwrStatus = FALSE;
	}
}
/****************************************************************************/
void Init_IdleInfo()
{
	FLASH_SETTINGS_T user_settings;
	Get_UserSettings(&user_settings);

	idle_info.state = WAITING_IDLE_TIMEOUT_STATE;
	idle_info.u32_totalIdleTime = 0;
	idle_info.u16_idleTimeBetweenMessage = 0;
	Set_Timer(&(idle_info.idle_alarm_timer), user_settings.u32_maxIdleTime * SECOND);
	Set_Timer(&idle_info.idle_secs_counter_timer, SECOND);
}
/****************************************************************************/
void Init_SpeedLimitInfo()
{
	speed_limit_info.state = BELOW_SPEED_LIMIT_STATE;
	speed_limit_info.u32_totalSpeedViolationTime = 0;
    Set_Timer(&speed_limit_info.speed_secs_counter_timer, SECOND);
}
/****************************************************************************/
void Init_StopInfo()
{
	FLASH_SETTINGS_T user_settings;
	Get_UserSettings(&user_settings);

	if(Get_IgnitionPinStatus())
		stop_info.state = NOT_STOPPED_STATE;
	else
		stop_info.state = STOPPED_STATE;

	stop_info.u32_totalStopTime = 0;
	Set_Timer(&stop_info.stop_alarm_timer, user_settings.u32_maxStopTime * SECOND);
//	Set_Timer(&stop_info.stop_secs_counter_timer, 100);
}
/***************************************************************************/
void Init_EventInfo()
{
	memset(&event_info, 0, sizeof(event_info));
}
/****************************************************************************/
TIMER_TICK_T GetTimeDifference(TIMER_TICK_T startTime, TIMER_TICK_T endTime)
{
	if(endTime >= startTime)
		return endTime - startTime;
	else
		return (0xFFFFFFFF - startTime + endTime);
}
/**************************************************************/
uint32_t GetStatusWord()
{
	uint32_t u32_status = 0;

	if(Get_IgnitionStatus())
		SetBit(&u32_status, (uint32_t)IGNITION_BIT_POS);
	else
		ResetBit(&u32_status, (uint32_t)IGNITION_BIT_POS);
/*
	if(status_info.speed_status == ABOVE_SPEED_LIMIT)
		SetBit(&u32_status, (uint32_t)SPEED_LIMIT_VIOLATED_BIT_POS);
	else
		ResetBit(&u32_status, (uint32_t)SPEED_LIMIT_VIOLATED_BIT_POS);

	if(status_info.power_status == STATUS_BATTERY_POWERED)
		SetBit(&u32_status, (uint32_t)POWER_STATUS_BIT);
	else
		ResetBit(&u32_status, (uint32_t)POWER_STATUS_BIT);*/

	return u32_status;
}
/************************************************************/
void SpeedLimitTestFunction()
{
	static uint8_t speed_test_state = 1;

	switch(speed_test_state)
	{
	/*	case 1:
		if(mn_timer_expired(&dummy_timer)){
			u16_speedKmTest +=10;
			if(u16_speedKmTest == 400){
				speed_test_state = 3;
				Set_Timer(&dummy_timer, 1500);
			}
			else
				Set_Timer(&dummy_timer, 10);
		}
		break;

		case 2:
		if(mn_timer_expired(&dummy_timer)){
			u16_speedKmTest-=10;
			if(u16_speedKmTest == 0){
				PRINT_K("Waiting Idle");
				speed_test_state = 4;
				Set_Timer(&dummy_timer, 300);
			}
			else
				Set_Timer(&dummy_timer, 10);
		}
		break;

		case 3:
		if(mn_timer_expired(&dummy_timer)){
			speed_test_state = 2;
			Set_Timer(&dummy_timer, 10);
		}
		break;

		case 4:
		if(mn_timer_expired(&dummy_timer)){
			Set_Timer(&dummy_timer, 10);
			speed_test_state = 1;
		}
		break;*/

	}
}
/********************************************************************/
bool Get_IgnitionPinStatus()
{
	if(!Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0,IGN_DETECT_PIN))
		return TRUE;   /* ignited */
	else
		return FALSE;  /* not ignited */

}
/*********************************************************************/
bool Get_SimDetectPinStatus()
{
	if(Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0,SIM_DETECT_PIN))
		return TRUE;     /* sim inserted */
	else
		return FALSE;
}
/*********************************************************************/
bool Get_PowerDetectPinStatus()
{
	if(Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0,POWER_DETECT_PIN))
		return TRUE;   /* power */
	else
		return FALSE;  /* battery */

}
/***********************************************************************/
void Clear_Events()
{
	memset(&event_info, 0, sizeof(event_info));
}
/***********************************************************************/
TIMER_TICK_T GetInterMessageIdleTime()
{
	return idle_info.u16_idleTimeBetweenMessage;
}
/***********************************************************************/
TIMER_TICK_T GetTotalIdleTime()
{
	return idle_info.u32_totalIdleTime;
}
/************************************************************************/
TIMER_TICK_T GetStopTime()
{
	return stop_info.u32_totalStopTime;
}
/************************************************************************/
TIMER_TICK_T GetSpeedViolationDuration()
{
	return speed_limit_info.u32_totalSpeedViolationTime;
}
/***********************************************************************/
void ResetSpeedViolationTime()
{
	speed_limit_info.u32_totalSpeedViolationTime = 0;
}
/***********************************************************************/
void ResetTotalIdleTime()
{
	idle_info.u32_totalIdleTime = 0;
}
/*************************************************************************/
void ResetStopTime()
{
	stop_info.u32_totalStopTime = 0;
}

