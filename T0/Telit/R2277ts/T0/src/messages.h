/*
 * messages.h
 *
 *  Created on: 30 Tem 2015
 *      Author: admin
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#define MAX_T_MESSAGE_SIZE             256

typedef enum
{
	DIGITAL_INPUT1_PASSIVE,
	DIGITAL_INPUT1_ACTIVE,
	DIGITAL_INPUT2_PASSIVE,
	DIGITAL_INPUT2_ACTIVE,
	SPEED_LIMIT_VIOLATION,
	IDLE_TIME_INDICATION,
	RFID_CARD_READING,
	CANBUS_DATA_READY,
	TEMP_SENSOR_DATA,
	MAX_STOP_TIME_EXCEEDED,
	EXT_POWER_STATUS_CHANGE,
	BATTERY_LEVEL,
	MAX_IDLE_TIME_EXCEEDED,
	OFFLINE_DATA = 19,
	ANALOG_INPUT_READING,
	GARMIN_NAVIGATION,
	CELL_ID,
	IGNITION_DEACTIVATED,
	IGNITION_ACTIVATED,
	TRIP_INFO,
	GPS_INFO,
	ROAMING_DATA,
	SIM_REMOVED,
	INTER_MESSAGE_IDLE_TIME
}MESSAGE_TYPE_T;

typedef struct COMMAND_RESPONSE
{
	char buffer[MAX_T_MESSAGE_SIZE];
	bool b_needToReset;
} COMMAND_RESPONSE_T;

#define SW_VERSION  "R2277"
#define BUILD_DATE  __DATE__



#if  defined(T0_QUECTEL_M66)
#define GSM_MODULE_CODE      "q"
#elif defined(T0_TELIT_GL865)
#define GSM_MODULE_CODE      "t"
#elif defined(T0_SIMCOM_SIM800C)
#define GSM_MODULE_CODE      "s"
#endif


#define      T_MESSAGE_HEADER       "[T"
#define      T_MESSAGE_FOOTER       "]"

#define VERSION          (SW_VERSION GSM_MODULE_CODE GPS_MODULE_CODE";"BUILD_DATE";")
//#define VERSION       (";"SW_VERSION MODULE_CODE";"BUILD_DATE";")
#define IMEI_LEN              15
#define IMSI_LEN              15
#define DATE_LEN              6
#define TIME_LEN              6

int32_t Trio_PrepareTMessage(char *, bool);
int32_t PrepareBlockageResponse(char *bufPtr);
void Trio_PrepareTMessageBody(char *, bool);
void Trio_BeginTMessage(char *);
void Trio_EndTMessage(char *);
int32_t Trio_PreparePingMessage(char *pBuffer, const char *imei_no);
int32_t Trio_PrepareSTMessage(char *pBuffer);
void Trio_PrepareTMessageExtension(char *, MESSAGE_TYPE_T);
void Trio_BeginTMessageExtension(char *u8_tMessage);
void Trio_AddTMessageExtensionSeperator(char *u8_tMessage);
void StartSendingMessage(int);
void Trio_StartTripInfoMessage(char *bufPtr);
#endif /* MESSAGES_H_ */
