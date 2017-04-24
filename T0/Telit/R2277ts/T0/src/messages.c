#include <string.h>
#include <stdlib.h>
#include "bsp.h"
#include "board.h"
#include "timer.h"
#include "gps.h"
#include "gsm.h"
#include "messages.h"
#include "settings.h"
#include "status.h"
#include "utils.h"
#include "bootloader.h"
/* Function prototypes*/
char get_ewns_code(char latitude, char longtitude);

char * Add_GpsDateTimeInfo(char *buffer, RMC_MESSAGE_T *rmc_info);
char * Add_GpsCoordinatesInfo(char *bufPtr, RMC_MESSAGE_T *rmc_info);
char * Add_GpsSpeedInfo(char * bufPtr,  RMC_MESSAGE_T *rmc_info);
char * Add_GpsCourseInfo(char * bufPtr,  RMC_MESSAGE_T *rmc_info);
char * Add_GpsEwnsInfo(char * bufPtr,  RMC_MESSAGE_T *rmc_info);

/****************************************************/
int32_t Trio_PreparePingMessage(char *pBuffer, const char *imei_no)
{
/*	GSM_INFO_T gsm_info;

	Get_GsmInfo(&gsm_info);*/

	memset(pBuffer, 0, MAX_T_MESSAGE_SIZE);

	strcat(pBuffer, "[P;");
	strcat(pBuffer, imei_no);
	strcat(pBuffer, "70");
	strcat(pBuffer, "]");

	return strlen(pBuffer);
}
/*****************************************************************************/
/** Fills the message buffer sent by pBuffer parameter and returns
 *  the number of bytes in formatted message
 *
 *  c_buffer  [in out]     buffer for the message
 *
 *  return number of bytes in the buffer
 ******************************************************************************/
int32_t Trio_PrepareSTMessage(char *pBuffer)
{
	GSM_INFO_T gsm_info;
	uint32_t resetStatusReg;

	*pBuffer = '\0';

	Get_GsmInfo(&gsm_info);
	strcat(pBuffer, "[ST;");
	strcat(pBuffer,gsm_info.imei_no);
	strcat(pBuffer,";T0-");
	strcat(pBuffer,VERSION);
	strcat(pBuffer,gsm_info.imsi_no);

	resetStatusReg =  LPC_SYSCTL->SYSRSTSTAT;
	LPC_SYSCTL->SYSRSTSTAT = 0xFF;
	switch(resetStatusReg)
	{
		case SYSCTL_RST_POR:
		strcat(pBuffer, ";PORRST]");
		break;

		case SYSCTL_RST_EXTRST:
		strcat(pBuffer, ";EXTRST]");
		break;

		case SYSCTL_RST_WDT:
		strcat(pBuffer, ";WDTRST]");
		break;

		case SYSCTL_RST_BOD:
		strcat(pBuffer, ";BODRST]");
		break;

		case SYSCTL_RST_SYSRST:
		strcat(pBuffer, ";SYSRST]");
		break;

		default:
		strcat(pBuffer, ";RECONNECT]");
		break;

	}

	return strlen(pBuffer);
}
/***********************************************************/
void Trio_PrepareTMessageBody(char *u8_tMessage, bool b_online)
{
	GSM_INFO_T gsm_info;
	RMC_MESSAGE_T rmc_info;
	GPS_POSITION_DATA_T position_info;
	uint32_t u32_km = 0;
	uint32_t u32_status;
	float mileage;
	char buffer2[10];


	Get_GsmInfo(&gsm_info);
	//Get_UserSettings(&user_settings);
	Get_RMCInfo(&rmc_info);
	u8_tMessage = strchr(u8_tMessage, '\0');

	if(b_online){
		strcat(u8_tMessage, gsm_info.imei_no);
		u8_tMessage = strchr(u8_tMessage, '\0');
	}
	u8_tMessage = Add_GpsDateTimeInfo(u8_tMessage, &rmc_info);
	u8_tMessage = Add_GpsCoordinatesInfo(u8_tMessage, &rmc_info);
	u8_tMessage = Add_GpsSpeedInfo(u8_tMessage, &rmc_info);
	u8_tMessage = Add_GpsCourseInfo(u8_tMessage, &rmc_info);
	u8_tMessage = Add_GpsEwnsInfo(u8_tMessage, &rmc_info);

	/* date */
/*	 (*u8_tMessage++) = (rmc_info.day/10)   + '0';
	 (*u8_tMessage++) = (rmc_info.day%10)   + '0';
	 (*u8_tMessage++) = (rmc_info.month/10) + '0';
	 (*u8_tMessage++) = (rmc_info.month%10) + '0';
	 (*u8_tMessage++) = (rmc_info.year/10)  + '0';
	 (*u8_tMessage++) = (rmc_info.year%10)  + '0';*/

	/* time */
	/* (*u8_tMessage++) =  (rmc_info.utc_hour/10)   + '0';
	 (*u8_tMessage++) =  (rmc_info.utc_hour%10)   + '0';
	 (*u8_tMessage++) =  (rmc_info.utc_minute/10) + '0';
	 (*u8_tMessage++) =  (rmc_info.utc_minute%10) + '0';
	 (*u8_tMessage++) =  (rmc_info.utc_second/10) + '0';
	 (*u8_tMessage++) =  (rmc_info.utc_second%10) + '0';*/

	/* latitude */
/*	 (*u8_tMessage++) = (rmc_info.coords.latitude.degrees /10)   + '0';
	 (*u8_tMessage++) = (rmc_info.coords.latitude.degrees %10)   + '0';
	 (*u8_tMessage++) = (rmc_info.coords.latitude.minutes /10)   + '0';
	 (*u8_tMessage++) = (rmc_info.coords.latitude.minutes %10)   + '0';*/

	//rmc_info.coords.latitude.minutes_frac /= 10;
/*	 (*u8_tMessage++) =  (rmc_info.coords.latitude.minutes_frac /1000)       + '0';
	 (*u8_tMessage++) =  (rmc_info.coords.latitude.minutes_frac %1000 /100)  + '0';
	 (*u8_tMessage++) =  (rmc_info.coords.latitude.minutes_frac %100)/10     + '0';
	 (*u8_tMessage++) =  (rmc_info.coords.latitude.minutes_frac %10)         + '0';*/

	/* longitude */
/*	 (*u8_tMessage++) = (rmc_info.coords.longitude.degrees /100)      + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.degrees %100) /10  + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.degrees %10)       + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.minutes /10)       + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.minutes %10)       + '0';*/

	//rmc_info.coords.longitude.minutes_frac /=10;
/*	 (*u8_tMessage++) = (rmc_info.coords.longitude.minutes_frac /1000)       + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.minutes_frac %1000) /100  + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.minutes_frac %100)/10     + '0';
	 (*u8_tMessage++) = (rmc_info.coords.longitude.minutes_frac %10)         + '0';*/

	 /* speed */
/*	 u32_speedKm = rmc_info.speed*1852/1000;
	 Hex2Str(buffer2, u32_speedKm);
	 buffer2[8] ='\0';
	 strcat(u8_tMessage, &buffer2[6]);*/

	 /* course */
	/* Hex2Str(buffer2,rmc_info.course);
	 buffer2[8] = '\0';
	 strcat(u8_tMessage, &buffer2[5]);
	 u8_tMessage = strchr(u8_tMessage, '\0');*/

/*	(*u8_tMessage++) = get_ewns_code(
			rmc_info.coords.latitude.indicator,
			rmc_info.coords.longitude.indicator);
*/
	/* insert 2 byte status information */
     u32_status = GetStatusWord();
    (*u8_tMessage++) = WORD32_BYTE1(u32_status) + '0';
    (*u8_tMessage++) = WORD32_BYTE0(u32_status) + '0';

	Get_PositionInfo(&position_info);
	mileage = position_info.distance / 1000;  /* convert to km */
	u32_km = Get_FlashKmValue();

	itoa((uint32_t)mileage + u32_km, buffer2, 10);

	/*PRINT_K("Total km...");
	PRINT_K(buffer2);
	PRINT_K("\n");*/

	Hex2Str(buffer2,(uint32_t)(mileage + u32_km));
	buffer2[8] = '\0';
	strcat(u8_tMessage, &buffer2[3]);

	 /* signal power*/
	itoa(gsm_info.csq, buffer2, 10);

	if(strlen(buffer2)  == 1)
		strcat(u8_tMessage, "0");
	strcat(u8_tMessage, buffer2);


//	return  strlen(u8_tMessage);
}

/***************************************************************************/
char get_ewns_code(char latitude, char longtitude)
{
	if(latitude == 'N' && longtitude == 'E') return '0';
	else if(latitude == 'N' && longtitude == 'W') return '1';
	else if(latitude == 'S' && longtitude == 'E') return '2';
	else if(latitude == 'S' && longtitude == 'W') return '3';
	else return '4';   /* return invalid direction code */
}
/****************************************************************************/
void Trio_BeginTMessage(char *u8_tMessage)
{
	memset(u8_tMessage, 0,MAX_T_MESSAGE_SIZE);
	strcat(u8_tMessage,"[T");
}
/*****************************************************************************/
void Trio_EndTMessage(char *u8_tMessage)
{

	strcat(u8_tMessage,"]");

	//return strlen(u8_tMessage);
}
/*****************************************************************************/
int32_t Trio_PrepareTMessage(char *u8_tMessage, bool b_online)
{

	EVENT_INFO_T event_info;

	Get_EventInfo(&event_info);

	Trio_BeginTMessage(u8_tMessage);
	Trio_PrepareTMessageBody(u8_tMessage, b_online);
	Trio_BeginTMessageExtension(u8_tMessage);
	Trio_PrepareTMessageExtension(u8_tMessage, GPS_INFO);

	if(GetInterMessageIdleTime() > 0){
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, INTER_MESSAGE_IDLE_TIME);
		ResetInterMessageIdleTime();
	}
	if(event_info.event_ignition_changed_on_to_off) {
	//	PRINT_K("\r\nAlarm:Ignition status changed\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, IGNITION_DEACTIVATED);
		 /* trip finished */
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, TRIP_INFO);
	//	event_info.event_ignition_status_changed = RESET;
	}
	if(event_info.event_ignition_changed_off_to_on) {
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, IGNITION_ACTIVATED);
	}
	if(event_info.event_idle_time_started) {
	//	PRINT_K("\r\nAlarm: Idle time Started Alarm\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, MAX_IDLE_TIME_EXCEEDED);
	//	event_info.event_idle_time_started = RESET;
	}
	if(event_info.event_idle_time_finished) {
	//	PRINT_K("\r\nAlarm: Idle time finished alarm\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, MAX_IDLE_TIME_EXCEEDED);
	//	event_info.event_idle_time_finished = RESET;
	}
	if(event_info.event_stop_time_started) {
	//	PRINT_K("\r\nAlarm: Stop time started\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
	    Trio_PrepareTMessageExtension(u8_tMessage, MAX_STOP_TIME_EXCEEDED);
	//    event_info.event_stop_time_started = RESET;
	}
	if(event_info.event_stop_time_finished) {
	//	PRINT_K("\r\nAlarm: Stop time finished\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, MAX_STOP_TIME_EXCEEDED);
	//	event_info.event_stop_time_finished = RESET;
	}
	if(event_info.event_speed_limit_violation_started) {
	//	PRINT_K("\r\nAlarm: Speed limit violation started\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, SPEED_LIMIT_VIOLATION);
	//	event_info.event_speed_limit_violation_started = RESET;
	}
	if(event_info.event_speed_limit_violation_finished) {
		//PRINT_K("\r\nAlarm: Speed limit violation finished\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, SPEED_LIMIT_VIOLATION);
	//	event_info.event_speed_limit_violation_finished = RESET;
	}
	if(event_info.event_sim_card_removed){
	//	PRINT_K("\r\nAlarm: SIM card removed\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, SIM_REMOVED);
	//	event_info.event_sim_card_removed = RESET;
	}
	if(event_info.event_power_status_changed) {
	//	PRINT_K("\r\nAlarm: Power status changed\r\n");
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, EXT_POWER_STATUS_CHANGE);
		//event_info.event_power_status_changed = RESET;
	}
	/* if main power supply is disconnected add power status to T message
	 */
	if(!Get_PowerDetectPinStatus()){
		Trio_AddTMessageExtensionSeperator(u8_tMessage);
		Trio_PrepareTMessageExtension(u8_tMessage, BATTERY_LEVEL);
	}
	Trio_EndTMessage(u8_tMessage);
	Clear_Events();


	return strlen(u8_tMessage);
}
/***********************************************************************************/
void Trio_StartTripInfoMessage(char *bufPtr)
{
	RMC_MESSAGE_T rmc_info;
	Get_RMCInfo(&rmc_info);
//	char *msg;

//	msg = bufPtr;

	bufPtr = Add_GpsDateTimeInfo(bufPtr, &rmc_info);
	bufPtr = Add_GpsCoordinatesInfo(bufPtr, &rmc_info);
	bufPtr = Add_GpsSpeedInfo(bufPtr, &rmc_info);
	bufPtr = Add_GpsCourseInfo(bufPtr, &rmc_info);
	bufPtr = Add_GpsEwnsInfo(bufPtr, &rmc_info);


	/* date */
/*	(*bufPtr++) = (rmc_info.day/10)   + '0';
	(*bufPtr++) = (rmc_info.day%10)   + '0';
	(*bufPtr++) = (rmc_info.month/10) + '0';
	(*bufPtr++) = (rmc_info.month%10) + '0';
	(*bufPtr++) = (rmc_info.year/10)  + '0';
	(*bufPtr++) = (rmc_info.year%10)  + '0';*/

	/* time */
/*	(*bufPtr++) =  (rmc_info.utc_hour/10)   + '0';
	(*bufPtr++) =  (rmc_info.utc_hour%10)   + '0';
	(*bufPtr++) =  (rmc_info.utc_minute/10) + '0';
	(*bufPtr++) =  (rmc_info.utc_minute%10) + '0';
	(*bufPtr++) =  (rmc_info.utc_second/10) + '0';
	(*bufPtr++) =  (rmc_info.utc_second%10) + '0';*/

	/* latitude */
/*	(*bufPtr++) = (rmc_info.coords.latitude.degrees /10)   + '0';
	(*bufPtr++) = (rmc_info.coords.latitude.degrees %10)   + '0';
	(*bufPtr++) = (rmc_info.coords.latitude.minutes /10)   + '0';
	(*bufPtr++) = (rmc_info.coords.latitude.minutes %10)   + '0';

	(*bufPtr++) =  (rmc_info.coords.latitude.minutes_frac /1000)       + '0';
	(*bufPtr++) =  (rmc_info.coords.latitude.minutes_frac %1000 /100)  + '0';
	(*bufPtr++) =  (rmc_info.coords.latitude.minutes_frac %100)/10     + '0';
	(*bufPtr++) =  (rmc_info.coords.latitude.minutes_frac %10)         + '0';*/

	/* longitude */
/*	(*bufPtr++) = (rmc_info.coords.longitude.degrees /100)      + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.degrees %100) /10  + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.degrees %10)       + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.minutes /10)       + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.minutes %10)       + '0';

	(*bufPtr++) = (rmc_info.coords.longitude.minutes_frac /1000)       + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.minutes_frac %1000) /100  + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.minutes_frac %100)/10     + '0';
	(*bufPtr++) = (rmc_info.coords.longitude.minutes_frac %10)         + '0';*/

	/* speed */
/*	u32_speedKm = rmc_info.speed*1852/1000;
	Hex2Str(buffer, u32_speedKm);
	buffer[8] ='\0';
	strcat(bufPtr, &buffer[6]);*/

	/* course */
/*	Hex2Str(buffer,rmc_info.course);
	buffer[8] = '\0';
	strcat(bufPtr, &buffer[5]);
	bufPtr = strchr(bufPtr, '\0');*/
	/* ewns*/
/*	(*bufPtr++) = get_ewns_code(rmc_info.coords.latitude.indicator,
							    rmc_info.coords.longitude.indicator);

*/

}
/*************************************************************************************/
void Trio_PrepareTMessageExtension(char *u8_tMessage, MESSAGE_TYPE_T message_type)
{
	GSA_MESSAGE_T gsa_info;
	GGA_MESSAGE_T gga_info;
	EVENT_INFO_T event_info;
	TIMER_TICK_T idle;
	int i_batteryVoltage;
	char temp[16];

	//Get_StatusInfo(&status_info);
	Get_EventInfo(&event_info);
	switch(message_type)	{

		case OFFLINE_DATA:
		strcat(u8_tMessage, "019");
		break;

		case INTER_MESSAGE_IDLE_TIME:
		idle = GetInterMessageIdleTime();
		itoa(idle, temp, 16);
		strcat(u8_tMessage, "00F:");
		strcat(u8_tMessage, temp);
		break;

		case  GPS_INFO:
		/* PARAM 1*/
		Get_GSAInfo(&gsa_info);
		Get_GGAInfo(&gga_info);

		strcat(u8_tMessage, "025:");
		u8_tMessage = strchr(u8_tMessage, '\0');
		(*u8_tMessage++) = gsa_info.mode2 + '0';
		strcat(u8_tMessage, ",");

		/* PARAM 2*/
		itoa(gsa_info.pdop_int, temp, 10);
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ".");
		itoa(gsa_info.pdop_frac, temp, 10);
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ",");

		/* PARAM 3*/
		itoa(gsa_info.hdop_int, temp, 10);
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ".");
		itoa(gsa_info.hdop_frac, temp, 10);
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ",");

		/* PARAM 4*/
		itoa(gsa_info.vdop_int, temp, 10);
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ".");
		itoa(gsa_info.vdop_frac, temp, 10);
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ",");

		/* PARAM 5 */
		itoa(gga_info.satellites, temp, 10);
		strcat(u8_tMessage, temp);
		break;

		case MAX_IDLE_TIME_EXCEEDED:
		strcat(u8_tMessage, "00X");
		if(event_info.event_idle_time_finished){
			strcat(u8_tMessage, ":");
		/*	Hex2Str(temp, GetIdleTime());
			temp[8] = '\0';*/
			itoa(GetTotalIdleTime(),temp,16);
			ResetTotalIdleTime();
			strcat(u8_tMessage, temp);
		}
		break;

		case MAX_STOP_TIME_EXCEEDED:
		strcat(u8_tMessage, "00C");
		if(event_info.event_stop_time_finished){
			strcat(u8_tMessage, ":");
			/*Hex2Str(temp, GetStopTime());
			temp[8] = '\0';*/
			itoa(GetStopTime(),temp,16);
			ResetStopTime();
			strcat(u8_tMessage, temp);
		}
		break;

		case SPEED_LIMIT_VIOLATION:
		strcat(u8_tMessage, "005:");
		if(event_info.event_speed_limit_violation_finished){
			strcat(u8_tMessage, "0,");
			itoa(GetSpeedViolationDuration(), temp, 16);
			ResetSpeedViolationTime();
			strcat(u8_tMessage, temp);
		}
		else
			strcat(u8_tMessage, "1");
		break;

		case SIM_REMOVED:
		strcat(u8_tMessage, "027");
		break;

		case EXT_POWER_STATUS_CHANGE:
		if(Get_PowerDetectPinStatus())
			strcat(u8_tMessage, "00D:1");
		else
			strcat(u8_tMessage, "00D:0");
		break;

		case IGNITION_ACTIVATED:
		strcat(u8_tMessage, "024:1");
		break;

		case IGNITION_DEACTIVATED:
		strcat(u8_tMessage, "024:0");
		break;

		case TRIP_INFO:
		strcat(u8_tMessage, "026:");
		strcat(u8_tMessage, Get_TripStartInfoBufferPtr());   /* PARAM 1*/
		strcat(u8_tMessage, ",");
		itoa(Get_TripDistance(), temp, 10);   /* PARAM 2*/
		strcat(u8_tMessage, temp);
		strcat(u8_tMessage, ",0,");           /* PARAM 3 ignored */
		itoa(Get_TripIdleTime(), temp, 10);   /* PARAM 4*/
		strcat(u8_tMessage, temp);
		break;

		case BATTERY_LEVEL:
		strcat(u8_tMessage, "00E:");
		itoa(GetBatteryVoltage(), temp, 10);
		strcat(u8_tMessage, temp);
		break;

	}
}
/*******************************************************************************/
void Trio_BeginTMessageExtension(char *u8_tMessage)
{
	strcat(u8_tMessage, "-");
	u8_tMessage++;
}
/*******************************************************************************/
void Trio_AddTMessageExtensionSeperator(char *u8_tMessage)
{
	strcat(u8_tMessage, ";");
	u8_tMessage++;
}
/*********************************************************************************/
char * Add_GpsDateTimeInfo(char *bufPtr, RMC_MESSAGE_T *rmc_info)
{
	/* date*/
	(*bufPtr++) = (rmc_info->day/10)   + '0';
	(*bufPtr++) = (rmc_info->day%10)   + '0';
	(*bufPtr++) = (rmc_info->month/10) + '0';
	(*bufPtr++) = (rmc_info->month%10) + '0';
	(*bufPtr++) = (rmc_info->year/10)  + '0';
	(*bufPtr++) = (rmc_info->year%10)  + '0';

	/* time */
	(*bufPtr++) =  (rmc_info->utc_hour/10)   + '0';
	(*bufPtr++) =  (rmc_info->utc_hour%10)   + '0';
	(*bufPtr++) =  (rmc_info->utc_minute/10) + '0';
	(*bufPtr++) =  (rmc_info->utc_minute%10) + '0';
	(*bufPtr++) =  (rmc_info->utc_second/10) + '0';
	(*bufPtr++) =  (rmc_info->utc_second%10) + '0';

	return bufPtr;
}
/*********************************************************************************/
char * Add_GpsCoordinatesInfo(char *bufPtr, RMC_MESSAGE_T *rmc_info)
{
	/* latitude */
	(*bufPtr++) = (rmc_info->coords.latitude.degrees /10)   + '0';
	(*bufPtr++) = (rmc_info->coords.latitude.degrees %10)   + '0';
	(*bufPtr++) = (rmc_info->coords.latitude.minutes /10)   + '0';
	(*bufPtr++) = (rmc_info->coords.latitude.minutes %10)   + '0';

	(*bufPtr++) =  (rmc_info->coords.latitude.minutes_frac /1000)       + '0';
	(*bufPtr++) =  (rmc_info->coords.latitude.minutes_frac %1000 /100)  + '0';
	(*bufPtr++) =  (rmc_info->coords.latitude.minutes_frac %100)/10     + '0';
	(*bufPtr++) =  (rmc_info->coords.latitude.minutes_frac %10)         + '0';

	/* longitude */
	(*bufPtr++) = (rmc_info->coords.longitude.degrees /100)      + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.degrees %100) /10  + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.degrees %10)       + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.minutes /10)       + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.minutes %10)       + '0';

	(*bufPtr++) = (rmc_info->coords.longitude.minutes_frac /1000)       + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.minutes_frac %1000) /100  + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.minutes_frac %100)/10     + '0';
	(*bufPtr++) = (rmc_info->coords.longitude.minutes_frac %10)         + '0';
	return bufPtr;
}
/*************************************************************************************/
char * Add_GpsSpeedInfo(char *bufPtr,  RMC_MESSAGE_T *rmc_info)
{
	uint32_t u32_speedKm;
	char buffer[16];

	memset(buffer, 0, sizeof(buffer));

	u32_speedKm = rmc_info->speed*1852/1000;
	Hex2Str(buffer, u32_speedKm);
	strcat(bufPtr, &buffer[6]);
	return bufPtr;
}
/*************************************************************************************/
char * Add_GpsCourseInfo(char *bufPtr,  RMC_MESSAGE_T *rmc_info)
{
	char buffer[16];

	memset(buffer, 0, sizeof(buffer));

	Hex2Str(buffer,rmc_info->course);
	strcat(bufPtr, &buffer[5]);
	bufPtr = strchr(bufPtr, '\0');

	return bufPtr;

}
/*************************************************************************************/
char * Add_GpsEwnsInfo(char *bufPtr,  RMC_MESSAGE_T *rmc_info)
{
	(*bufPtr++) = get_ewns_code(rmc_info->coords.latitude.indicator,
							    rmc_info->coords.longitude.indicator);
	return bufPtr;
}

