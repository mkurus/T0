/*
 * status.h
 *
 *  Created on: 31 Tem 2015
 *      Author: admin
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "timer.h"

void PIN_INT0_IRQHandler();
void PIN_INT1_IRQHandler();
void PIN_INT2_IRQHandler();
void PIN_INT3_IRQHandler();
void PIN_INT4_IRQHandler();
void PIN_INT5_IRQHandler();
void PIN_INT6_IRQHandler();
void PIN_INT7_IRQHandler();

void Clear_Events();
void Trio_ConfigureSimDetectInterrupt();
void Trio_ConfigureIgnDetectInterrupt();
void Trio_ConfigureRelayGPIO();
void SetRoamingStatus(bool status);
bool Get_IgnitionPinStatus();
bool Get_IgnitionStatus();
bool Get_PowerDetectPinStatus();
bool Get_SimDetectPinStatus();
uint32_t GetStatusWord();
uint32_t Get_TripDistance();

uint32_t Get_TripIdleTime();
char * Get_TripStartInfoBufferPtr();
TIMER_TICK_T GetTimeDifference(TIMER_TICK_T startTime, TIMER_TICK_T endTime);
bool GetRoamingStatus();
bool Get_EventStatus();
bool Get_PowerStatus();
TIMER_TICK_T GetTotalIdleTime();
TIMER_TICK_T GetInterMessageIdleTime();
TIMER_TICK_T GetStopTime();
TIMER_TICK_T GetSpeedViolationDuration();
bool CheckSIMStatus();
void Trio_StatusTask();
void UpdateBatteryVoltageStatus(char *buffer);

typedef enum {
	IGNITION_BIT_POS,
	ROAMING_BIT_POS,
	GPS_BIT_POS,
	DIGITAL_INPUT1_BIT,
	DIGITAL_INPUT2_BIT,
	SPEED_LIMIT_VIOLATED_BIT_POS,
	POWER_STATUS_BIT
}STATUS_BITS;

typedef enum
{
	IDLE_STATE,
	NOT_IDLE_STATE,
	WAITING_IDLE_TIMEOUT_STATE
}IDLE_STATES_T;
typedef enum
{
	VEHICLE_STATUS_STOPPED,
	VEHICLE_STATUS_NOT_STOPPED,
	WAITING_VEHICLE_STOP_TIMEOUT
}STOP_STATUS_T;

typedef enum
{
	BELOW_SPEED_LIMIT,
	ABOVE_SPEED_LIMIT
}SPEED_STATUS_T;

typedef enum
{
	STATUS_BATTERY_POWERED,
	STATUS_EXT_POWERED
}POWER_STATUS_T;

typedef enum GPS_TRIP_STATUS{
	VALID_GPS_ON_IGNITION,
	INVALID_GPS_ON_IGNITION,
	VALID_GPS_ON_DEIGNITION,
	INVALID_GPS_ON_DEIGNITION
}GPS_TRIP_STATUS_T;

typedef struct STOP_TIME_INFO
{
	TIMER_INFO_T stop_timer;
	STOP_STATUS_T stop_status;
	TIMER_TICK_T stop_time;
}STOP_TIME_INFO_T;

typedef struct
{
	POWER_STATUS_T power_status:1;
	int batteryLevel;
}STATUS_INFO_T;

typedef struct EVENT_INFO
{
	uint32_t event_digtial_input1_low:1;
	uint32_t event_digtial_input1_high:1;
	uint32_t event_digtial_input2_low:1;
	uint32_t event_digital_input2_high:1;
	uint32_t event_speed_limit_violation_started:1;
	uint32_t event_speed_limit_violation_finished:1;
	uint32_t event_idle_time_started:1;
	uint32_t event_idle_time_finished:1;
	uint32_t event_stop_time_started:1;
	uint32_t event_stop_time_finished:1;
	uint32_t event_power_status_changed:1;
	uint32_t event_cell_id_changed:1;
	uint32_t event_ignition_changed_on_to_off:1;
	uint32_t event_ignition_changed_off_to_on:1;
	uint32_t event_sim_card_removed:1;
	uint32_t event_engine_blocked:1;
}EVENT_INFO_T;

GPS_TRIP_STATUS_T Get_TripGpsStatus();
void Init_TripInfo(GPS_TRIP_STATUS_T status);
void SetBlockage(bool blockageStatus);
void ResetTotalIdleTime();
void ResetInterMessageIdleTime();
void ResetStopTime();
void ResetSpeedViolationTime();
void Get_EventInfo(EVENT_INFO_T *);
void Get_StatusInfo(STATUS_INFO_T *);
void Init_StatusInfo();
void Init_EventInfo();
void Set_BlockageSource(COMMAND_SOURCE_T source);
void Get_BlockageInfo(BLOCKAGE_INFO_T *blockage);
void Set_BlockageTransID(char *transactionID);
void Get_BlockageTransID(char *transactionID);

#define   BLOCKAGE_PIN          19    /* drives blockage relay */
#define   SIM_DETECT_PIN        17    /* PIO_17 as SIM detection pin */
#define   IGN_DETECT_PIN        14    /* PIO_14 as Ignition detection pin */
#define   POWER_DETECT_PIN      22    /* External power detection pin */

#endif /* STATUS_H_ */
