/*
 * Settings.c

 *
 *  Created on: 30 Tem 2015
 *      Author: admin
 */

#include "board.h"
#include "bsp.h"
#include "at_commands.h"
#include "gps.h"
#include "messages.h"
#include "settings.h"
#include "status.h"
#include "gsm.h"
#include "spi.h"
#include "bootloader.h"
#include "utils.h"
#include "sst25.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
TCP_CONNECTION_INFO_T firmwareUpdateServerInfo_t;
FLASH_SETTINGS_T flash_settings;

#define CONFIG_PARAM_SEPERATOR     ";"
#define TRANSACTION_ID_SEPERATOR     "!"
const FLASH_SETTINGS_T default_settings = {
		"triomobil_apn",
		"apnusername",
		"apnpassword",
		"bodrum.triomobil.com",   /* dns adresi olacak = alacati.triomobil.com  */
		"6081",
	    "1234567890ABCDE",
		"ATR33",
		 TRUE,         /* keep blockage status in RAM */
		 TRUE,         /* roaming */
		 TRUE,         /* sms     */
		 FALSE,        /* blockage?  */
		 (1  * 60),    /* message period roaming/ignited */
		 (60 * 60),    /* message period roaming/not-ignited */
		  5,           /* message period not-roaming/ignited */
		 (60 * 60),    /* message period not-roaming/not-ignited */
		  120,                /* speed limit in km */
		  10,                 /* speed limit violation duration */
		 (10 * 60),           /* max. stop time */
	      90,                 /* max. idle time */
		  0xFFFFFFFF,         /* gps baud rate */
		  0x00000000          /* checksum  */
};

/* callback functions for flash write operations*/
COMMAND_RESULT_T update_server_and_port_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_message_period_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_roaming_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_sms_act_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_speed_limit_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_engine_blockage_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_km_counter_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_device_id(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T reset_device(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T send_sms_command(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T request_device_status(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T uncontrolled_engine_blockage(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T location_request(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_apn_setting(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_password(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_firmware(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T gps_cold_restart(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T erase_external_flash(char *const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_blockage_persistance(char * const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_idle_alarm_setting(char * const buffer, COMMAND_RESPONSE_T *response);
COMMAND_RESULT_T update_gps_baudrate_setting(char * const buffer, COMMAND_RESPONSE_T *response);
/*  get command callback functions */
COMMAND_RESULT_T get_healt_status(char *const buffer, COMMAND_RESPONSE_T *response);


COMMAND_TYPE_T Get_ConfigurationCommand(char *buffer);
void Get_ConfigurationParameter(char * const, char *, char *);
bool validate_connection_params(FLASH_SETTINGS_T *settings);
void Init_EchoInfo();

#define  VALID_SETTINGS_PAGE_SIGN_ADDR  		(249 * 4096)

#define  VALID_SETTINGS_SIGN                     0xAA55AA55
static   uint32_t u32_flashKmCounter = 0;
TCP_CONNECTION_INFO_T updateServerInfo_t;
bool requestConnectToUpdateServer = FALSE;
/*************************************************************************/
const SETTING_INFO_T settings_info[] =
{
	{ SERVER_AND_PORT_SETTING,       update_server_and_port_setting },
    { MESSAGE_PERIOD_SETTING,        update_message_period_setting },
	{ ROAMING_ACTIVATION,            update_roaming_setting },  /* kaldırılabilir */
/*	{ SMS_ACTIVATION,                update_sms_act_setting },*/
	{ IDLE_ALARM_SETTING,            update_idle_alarm_setting},
	{ SPEED_LIMIT_SETTING,           update_speed_limit_setting },
	{ ENGINE_BLOCKAGE_SETTING,       update_engine_blockage_setting},
	{ KM_COUNTER,                    update_km_counter_setting},
	{ DEVICE_RESET,                  reset_device },
	{ SEND_SMS,                      send_sms_command },
	{ DEVICE_ID_SETTING,             update_device_id },   /* kaldır */
	{ DEVICE_STATUS_REQUEST,         request_device_status },   /* kaldır*/
	{ LOCATION_REQUEST,              location_request },
	{ APN_SETTING,                   update_apn_setting },
	{ CHANGE_PASSWORD,               update_password },
	{ FIRMWARE_UPDATE_REQUEST,       update_firmware},
//	{ GPS_COLD_RESTART,              gps_cold_restart},
	{ ERASE_EXT_FLASH,               erase_external_flash},
//	{ SET_BLOCKAGE_PERSISTANCE,      update_blockage_persistance},  /* kaldır */
	{ GPS_BAUDRATE_SETTING,  		 update_gps_baudrate_setting}
};

const GET_CMD_INFO_T get_cmd[] =
{
	{ DEVICE_HEALT_STATUS,      get_healt_status}

};

/*********************************************************************************/
COMMAND_RESULT_T ParseConfigurationString(char *const buffer, COMMAND_RESPONSE_T *response, COMMAND_SOURCE_T cmdSource)
{
	COMMAND_TYPE_T command;
	COMMAND_RESULT_T result = REPLY_DO_NOT_SEND;
	uint32_t i;

	command = Get_ConfigurationCommand(buffer);
	for(i = 0; i < sizeof(settings_info) / sizeof(settings_info[0]); i++ ){
		if(command == settings_info[i].command){
			result = settings_info[i].update_callback(buffer, response);
			if(command == ENGINE_BLOCKAGE_SETTING)
				Set_BlockageSource(cmdSource);
			break;
		}
	}
	return result;
}
/***********************************************************************/
COMMAND_RESULT_T ParseGetCommand(char *const buffer, COMMAND_RESPONSE_T *response, COMMAND_SOURCE_T cmdSource)
{
	GET_COMMAND_TYPE command;
	COMMAND_RESULT_T result = REPLY_DO_NOT_SEND;
	uint32_t i;

	command = Get_ConfigurationCommand(buffer);
	for(i = 0; i < sizeof(get_cmd) / sizeof(get_cmd[0]); i++ ){
		if(command == get_cmd[i].command){
			result = get_cmd[i].get_callback(buffer, response);
			break;
		}
	}
	return result;
}
/***********************************************************************/
COMMAND_RESULT_T get_healt_status(char *const buffer, COMMAND_RESPONSE_T *response)
{
	memset(response->buffer, 0, MAX_T_MESSAGE_SIZE);
	BeginGetResponse(response->buffer, DEVICE_HEALT_STATUS);
	strcat(response->buffer, ";");
	strcat(response->buffer, SW_VERSION);
	strcat(response->buffer, GSM_MODULE_CODE);
	strcat(response->buffer, GPS_MODULE_CODE);
//	strcat(response->buffer, ";");
	if(Get_GsmHealtStatus())
		AddStringToEchoPacket(response->buffer, "GSM:1");
	else
		AddStringToEchoPacket(response->buffer, "GSM:0");

	//strcat(response->buffer, ";");

	if(Get_GpsHealtStatus())
		AddStringToEchoPacket(response->buffer, "GPS:1");
	else
		AddStringToEchoPacket(response->buffer, "GPS:0");

//	strcat(response->buffer, ";");

	if(Get_FlashHealtStatus())
		AddStringToEchoPacket(response->buffer, "FLASH:1");
	else
		AddStringToEchoPacket(response->buffer, "FLASH:0");

	CloseEchoPacket(response->buffer);
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;
}

/***********************************************************************/
COMMAND_TYPE_T Get_ConfigurationCommand(char *buffer)
{
	 char *pTemp;

	 pTemp = strtok (buffer, CONFIG_PARAM_SEPERATOR);
	 if(pTemp != NULL){
		 return (COMMAND_TYPE_T)atoi(buffer);
	 }
	 else
		 return (COMMAND_TYPE_T)0;
}
/************************************************************************/
void Get_ConfigurationParameter(char * const in_buffer, char *out_buffer, char *seperator)
{
	char *pTemp;
	pTemp = strtok(NULL, seperator);
	strcpy(out_buffer, pTemp);
}
/************************************************************************/
COMMAND_RESULT_T update_server_and_port_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];

	/* extract server ip address*/
	Get_ConfigurationParameter(NULL, flash_settings.server_ip, CONFIG_PARAM_SEPERATOR);
	/* extract port number*/
	Get_ConfigurationParameter(NULL, flash_settings.server_port, CONFIG_PARAM_SEPERATOR);
	/* extract transaction id*/
	Get_ConfigurationParameter(NULL, transactionID, TRANSACTION_ID_SEPERATOR);

	Update_Settings();

	BeginEchoPacket(response->buffer, SERVER_AND_PORT_SETTING);
	AddStringToEchoPacket(response->buffer, flash_settings.server_ip);
	AddStringToEchoPacket(response->buffer, flash_settings.server_port);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	response->b_needToReset = TRUE;
	return REPLY_ECHO_PACKET;
}
/*************************************************************************/
COMMAND_RESULT_T update_message_period_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[16];
	char command;
	uint32_t val1, val2;
	bool valid = FALSE;
	char transactionID[16];

/*	PRINT_K("\r\nIn update_message_period_setting\r\n");
	PRINT_K(buffer);*/

	Get_ConfigurationParameter(NULL, temp, CONFIG_PARAM_SEPERATOR);
	command = temp[0];
	Get_ConfigurationParameter(NULL, temp, CONFIG_PARAM_SEPERATOR);
	val1 = atoi(temp);
	Get_ConfigurationParameter(NULL, temp, CONFIG_PARAM_SEPERATOR);
	val2 = atoi(temp);
	Get_ConfigurationParameter(NULL, transactionID, TRANSACTION_ID_SEPERATOR);

	if(command == '0'){
		flash_settings.msg_period_not_roaming_ignited = val1;
		flash_settings.msg_period_not_roaming_not_ignited = val2;
		valid = TRUE;
	}
	else if(command == '1') {
		flash_settings.msg_period_roaming_ignited = val1;
		flash_settings.msg_period_roaming_not_ignited = val2;
		valid = TRUE;
	}
	if(valid){
		Update_Settings();
		ReloadPeriodicDataSendTimer();
		BeginEchoPacket(response->buffer, MESSAGE_PERIOD_SETTING);
		temp[0] = command;
		temp[1] = '\0';
		AddStringToEchoPacket(response->buffer, temp);
		if(command == '0'){
			itoa(flash_settings.msg_period_not_roaming_ignited, temp, 10);
			AddStringToEchoPacket(response->buffer, temp);
			itoa(flash_settings.msg_period_not_roaming_not_ignited, temp, 10);
			AddStringToEchoPacket(response->buffer, temp);
		}
		else{
			itoa(flash_settings.msg_period_roaming_ignited, temp, 10);
			AddStringToEchoPacket(response->buffer, temp);
			itoa(flash_settings.msg_period_roaming_not_ignited, temp, 10);
			AddStringToEchoPacket(response->buffer, temp);
		}
		AddStringToEchoPacket(response->buffer, transactionID);
		CloseEchoPacket(response->buffer);
		response->b_needToReset = FALSE;
		return REPLY_ECHO_PACKET;
	}
	else
		return REPLY_DO_NOT_SEND;
}
/***********************************************************************/
COMMAND_RESULT_T update_roaming_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[2];
	char command;
	char transactionID[16];

	Get_ConfigurationParameter(NULL, &command, CONFIG_PARAM_SEPERATOR);

	if(command == '0' || command == '1'){
		Get_ConfigurationParameter(NULL, transactionID, TRANSACTION_ID_SEPERATOR);
		flash_settings.roaming_activation = command - 0x30;
		Update_Settings();
		BeginEchoPacket(response->buffer, ROAMING_ACTIVATION);
		temp[0] = flash_settings.roaming_activation + 0x30;
		temp[1] = '\0';
		AddStringToEchoPacket(response->buffer, temp);
		CloseEchoPacket(response->buffer);
		response->b_needToReset = FALSE;
		return REPLY_ECHO_PACKET;
	}
	else
		return REPLY_DO_NOT_SEND;
}
/****************************************************************/
COMMAND_RESULT_T update_sms_act_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[2];

	Get_ConfigurationParameter(NULL, temp, CONFIG_PARAM_SEPERATOR);
	if(strcmp(temp, "0") == 0){
		flash_settings.sms_activation = FALSE;
		Update_Settings();
	}
	else if(strcmp(temp, "1") == 0){
		flash_settings.sms_activation = TRUE;
		Update_Settings();
	}

	return REPLY_DO_NOT_SEND;
}
/***************************************************************/
COMMAND_RESULT_T update_speed_limit_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	uint16_t u16_speedLimit;
	uint16_t u16_duration;
	char transactionID[16];
	char temp[8];

	/* extract speed limit*/
	Get_ConfigurationParameter(buffer, temp, CONFIG_PARAM_SEPERATOR);
	u16_speedLimit = atoi(temp);
	flash_settings.u16_speedLimit = u16_speedLimit;

    /* extract speed limit violation duration */
	Get_ConfigurationParameter(buffer, temp, CONFIG_PARAM_SEPERATOR);
	u16_duration = atoi(temp);
	flash_settings.u16_speedLimitViolationDuration = u16_duration;

	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);

	Update_Settings();

	BeginEchoPacket(response->buffer, SPEED_LIMIT_SETTING);
    itoa(flash_settings.u16_speedLimit, temp, 10);
	AddStringToEchoPacket(response->buffer, temp);
	itoa(flash_settings.u16_speedLimitViolationDuration, temp, 10);
	AddStringToEchoPacket(response->buffer, temp);

	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;
}
/*******************************************************************/
COMMAND_RESULT_T update_idle_alarm_setting(char * const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];
	char temp[16];

	Get_ConfigurationParameter(buffer, temp, CONFIG_PARAM_SEPERATOR);
	flash_settings.u32_maxIdleTime = atoi(temp);
	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
	Update_Settings();

	BeginEchoPacket(response->buffer, IDLE_ALARM_SETTING);
	itoa(flash_settings.u32_maxIdleTime, temp, 10);
	AddStringToEchoPacket(response->buffer, temp);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;
}
/*******************************************************************/
COMMAND_RESULT_T update_engine_blockage_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char param[2];
	char transactionID[16];
	bool success = false;

	Get_ConfigurationParameter(buffer, param, CONFIG_PARAM_SEPERATOR);
	if(strcmp(param, "0") == 0){
		flash_settings.engine_blockage = FALSE;
		Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
		PRINT_K("BLOCKAGE REMOVE REQUEST\n");

		if(!flash_settings.blockage_status_in_ram)
			Update_Settings();

		BeginEchoPacket(response->buffer, ENGINE_BLOCKAGE_SETTING);
		AddStringToEchoPacket(response->buffer, "0");   /* blockage removed*/
		Set_BlockageStatus(BLOCKAGE_REMOVE_REQUESTED);
		success = true;
	}
	else if(strcmp(param, "1") == 0){
		PRINT_K("BLOCKAGE REQUEST\n");
		flash_settings.engine_blockage = TRUE;
		Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);

		if(!flash_settings.blockage_status_in_ram)
			Update_Settings();

		BeginEchoPacket(response->buffer, ENGINE_BLOCKAGE_SETTING);
		AddStringToEchoPacket(response->buffer, "1");
		Set_BlockageStatus(BLOCKAGE_REQUESTED);
		success = true;
	}
	if(success){
		Set_BlockageTransID(transactionID);
		AddStringToEchoPacket(response->buffer, transactionID);
		CloseEchoPacket(response->buffer);
	    response->b_needToReset = FALSE;
	}
	return REPLY_ECHO_PACKET;
}
/*******************************************************************/
COMMAND_RESULT_T update_km_counter_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp1[64];
	char temp2[64];
	uint32_t u32_flashKmTempCounter;
	char transactionID[16];


	Get_ConfigurationParameter(buffer, temp1, CONFIG_PARAM_SEPERATOR);
	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
	strncpy(temp2, temp1, strlen(temp1) - 4);
	u32_flashKmTempCounter = atoi(temp2);

	if(UpdateKmCounter(u32_flashKmTempCounter) == true)
		u32_flashKmCounter = u32_flashKmTempCounter;

	BeginEchoPacket(response->buffer, KM_COUNTER);
	uitoa(u32_flashKmCounter, temp1, 10);
	strcat(temp1,"0000");
	AddStringToEchoPacket(response->buffer, temp1);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;
}
/*******************************************************************/
COMMAND_RESULT_T reset_device(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[2];
	Get_ConfigurationParameter(buffer, temp, CONFIG_PARAM_SEPERATOR);
	if(strcmp(temp, "1") == 0)
		NVIC_SystemReset();

	return REPLY_DO_NOT_SEND;
}
/*******************************************************************/
COMMAND_RESULT_T update_device_id(char *const buffer, COMMAND_RESPONSE_T *response)
{
	uint8_t i;
	char transactionID[16];

	Get_ConfigurationParameter(buffer, flash_settings.device_id, CONFIG_PARAM_SEPERATOR);
	for(i= 0; i< sizeof(flash_settings.device_id)- 1; i++){
		if(!isdigit(flash_settings.device_id[i])){
			return REPLY_DO_NOT_SEND;
		}
	}
	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
	Update_Settings();

	BeginEchoPacket(response->buffer, DEVICE_ID_SETTING);
	AddStringToEchoPacket(response->buffer, flash_settings.device_id);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;

}
/*********************************************************************/
COMMAND_RESULT_T request_device_status(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[2];

	GSM_INFO_T gsm_info;

	Get_GsmInfo(&gsm_info);

	Get_ConfigurationParameter(buffer, temp, CONFIG_PARAM_SEPERATOR);
	if(strcmp(temp, "1") == 0){
		Trio_PreparePingMessage(response->buffer, gsm_info.imei_no);
		response->b_needToReset = FALSE;
		return REPLY_ECHO_PACKET;
	}
	else
		return REPLY_DO_NOT_SEND;
}
/************************************************************/
/* not implemented n the current version                     /
*************************************************************/
COMMAND_RESULT_T uncontrolled_engine_blockage(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[2];

	Get_ConfigurationParameter(buffer, temp, CONFIG_PARAM_SEPERATOR);
	if(strcmp(temp, "0") == 0){
	//	flash_settings.uncontrolled_blockage = FALSE;
		Update_Settings();
	}
	else if(strcmp(temp, "1") == 0){
		//flash_settings.uncontrolled_blockage = TRUE;
		Update_Settings();
	}
	return REPLY_DO_NOT_SEND;
}
/********************************************************************/
/*  Send a T message in response to location request                 *
*********************************************************************/
COMMAND_RESULT_T location_request(char *const buffer, COMMAND_RESPONSE_T *response)
{
	PRINT_K("Location request\n");
	Trio_PrepareTMessage(response->buffer, TRUE);
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;
}
/********************************************************************/
COMMAND_RESULT_T update_apn_setting(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];
	//char *pBuf;

	Get_ConfigurationParameter(buffer, flash_settings.flash_apn, CONFIG_PARAM_SEPERATOR);
	Get_ConfigurationParameter(buffer, flash_settings.flash_apnusername, CONFIG_PARAM_SEPERATOR);
	Get_ConfigurationParameter(buffer, flash_settings.flash_apnpassword, CONFIG_PARAM_SEPERATOR);
	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
	Update_Settings();

	BeginEchoPacket(response->buffer, APN_SETTING);
	AddStringToEchoPacket(response->buffer, flash_settings.flash_apn);
	AddStringToEchoPacket(response->buffer, flash_settings.flash_apnusername);
	AddStringToEchoPacket(response->buffer, flash_settings.flash_apnpassword);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	response->b_needToReset = TRUE;
	return REPLY_ECHO_PACKET;
}
/********************************************************************/
COMMAND_RESULT_T update_password(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];
	char pswdBuf[MAX_SET_COMMAND_PSWD_LENGTH + 1];
//	char *pBuf;

	Get_ConfigurationParameter(buffer, pswdBuf, CONFIG_PARAM_SEPERATOR);

	if(strlen(pswdBuf) > MAX_SET_COMMAND_PSWD_LENGTH)
		return 0;
	else{
		strcpy(flash_settings.password, pswdBuf);
		Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
		Update_Settings();

		BeginEchoPacket(response->buffer, CHANGE_PASSWORD);
		AddStringToEchoPacket(response->buffer, flash_settings.password);
		AddStringToEchoPacket(response->buffer, transactionID);
		CloseEchoPacket(response->buffer);
		response->b_needToReset = FALSE;
		return REPLY_ECHO_PACKET;
	}
}
/********************************************************************/
COMMAND_RESULT_T send_sms_command(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];
	char smsBuf[MAX_SMS_LENGTH];
	char destAddr[DEST_ADDR_LENGTH];


	Get_ConfigurationParameter(buffer, destAddr, CONFIG_PARAM_SEPERATOR);          /* extract destination address */
	Get_ConfigurationParameter(buffer, smsBuf, CONFIG_PARAM_SEPERATOR);            /* extract sms content         */
	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);   /* extract transaction id      */

	response->b_needToReset = FALSE;
	PRINT_K("SMS Config\n");
	if(Send_SMS(destAddr, smsBuf, NULL) == TRUE){
		BeginEchoPacket(response->buffer, SEND_SMS);
		AddStringToEchoPacket(response->buffer, destAddr);
		AddStringToEchoPacket(response->buffer, smsBuf);
		AddStringToEchoPacket(response->buffer, transactionID);
		CloseEchoPacket(response->buffer);
		return REPLY_ECHO_PACKET;
	}
	return REPLY_DO_NOT_SEND;

}
/********************************************************************/
COMMAND_RESULT_T update_firmware(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char server_ip[128];
	char server_port[6];
	PRINT_K("Start Update\n");
	/* get firmware update server ip address*/
	Get_ConfigurationParameter(buffer, server_ip , CONFIG_PARAM_SEPERATOR);
	/* get firmware update TCP port number*/
	Get_ConfigurationParameter(buffer, server_port, CONFIG_PARAM_SEPERATOR);
	Set_ServerConnectionParameters(server_ip, server_port);
	requestConnectToUpdateServer = TRUE;
	return REPLY_CONNECT_TO_UPDATE_SERVER;
}
/********************************************************************/
COMMAND_RESULT_T  erase_external_flash(char *const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];

	SST25_ChipErase();
//	write_default_settings();
//	WriteInitialValuesToFlash();
//	Init_SPI_Cache();
//	Load_UserSettings();
//	Load_KmCounter();

	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);

	BeginEchoPacket(response->buffer, ERASE_EXT_FLASH);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);

	response->b_needToReset = TRUE;
	return REPLY_ECHO_PACKET;
}
/********************************************************************/
COMMAND_RESULT_T update_blockage_persistance(char * const buffer, COMMAND_RESPONSE_T *response)
{
	char temp[2];
	char command;
	char transactionID[16];

	Get_ConfigurationParameter(NULL, &command, CONFIG_PARAM_SEPERATOR);

	if(command == '0' || command == '1'){
		Get_ConfigurationParameter(NULL, transactionID, TRANSACTION_ID_SEPERATOR);
		flash_settings.blockage_status_in_ram = command - 0x30;
		Update_Settings();
		BeginEchoPacket(response->buffer, SET_BLOCKAGE_PERSISTANCE);
		temp[0] = flash_settings.blockage_status_in_ram + 0x30;
		temp[1] = '\0';
		AddStringToEchoPacket(response->buffer, temp);
		CloseEchoPacket(response->buffer);
		response->b_needToReset = FALSE;
		return REPLY_ECHO_PACKET;
	}
	else
		return REPLY_DO_NOT_SEND;
}
/********************************************************************/
COMMAND_RESULT_T update_gps_baudrate_setting(char * const buffer, COMMAND_RESPONSE_T *response)
{
	char transactionID[16];
	char gps_baudrate[16];

	Get_ConfigurationParameter(buffer, gps_baudrate, CONFIG_PARAM_SEPERATOR);
	Get_ConfigurationParameter(buffer, transactionID, TRANSACTION_ID_SEPERATOR);
	flash_settings.gps_baud_rate = atoi(gps_baudrate);
	Update_Settings();

	BeginEchoPacket(response->buffer, GPS_BAUDRATE_SETTING);
	itoa(flash_settings.gps_baud_rate, gps_baudrate, 10);

	AddStringToEchoPacket(response->buffer, gps_baudrate);
	AddStringToEchoPacket(response->buffer, transactionID);
	CloseEchoPacket(response->buffer);
	Trio_Init_GPS_UART();
	response->b_needToReset = FALSE;
	return REPLY_ECHO_PACKET;


}
/********************************************************************/
COMMAND_RESULT_T gps_cold_restart(char *const buffer, COMMAND_RESPONSE_T *response)
{
	PRINT_K("COLDSTART\n");
	send_mtk_command("$PMTK104*37\r\n");
	return REPLY_DO_NOT_SEND;
}
/*********************************************************************/
void Update_Settings()
{
	uint32_t chksum;
//	SST25_EraseSector(FIRST_FLASH_SETTINGS_ADDRESS / SST25_SECTOR_SIZE);

	chksum = calc_chksum((uint8_t *)&flash_settings, sizeof(flash_settings) - sizeof(flash_settings.chksum));
	flash_settings.chksum = chksum;

	SST25_WriteSR(0);
	SST25_WriteArray((uint8_t *)&flash_settings, sizeof(flash_settings), FIRST_FLASH_SETTINGS_ADDRESS);
	SST25_WriteSR(SST25_BP1);

	Load_UserSettings();   /* reload updated settings from FLASH to RAM*/

	PRINT_K("\nSETTINGS UPDATED\n");
}
/********************************************************************/
void Load_UserSettings()
{
	uint32_t chksum;


	SST25_Read(FIRST_FLASH_SETTINGS_ADDRESS, sizeof(FLASH_SETTINGS_T), (uint8_t *)&flash_settings);

	chksum = calc_chksum((uint8_t *)&flash_settings, sizeof(FLASH_SETTINGS_T) -sizeof(flash_settings.chksum));

	/*Print_Val("\nCalculated Checksum :",chksum );
	Print_Val("\nRead Checksum :",flash_settings.chksum);*/

	/* This section is only active in production programming */
	if(chksum != flash_settings.chksum) {
		PRINT_K("\nInvalid Settings\n");
		write_default_settings();
		/*memcpy(&flash_settings, &default_settings, sizeof(FLASH_SETTINGS_T));
		chksum = calc_chksum((uint32_t *)&flash_settings, (sizeof(flash_settings) / sizeof(uint32_t)) - 1);
		flash_settings.chksum = chksum;
		SST25_WriteSR(0);
		SST25_WriteArray((uint8_t *)&flash_settings, sizeof(FLASH_SETTINGS_T), FIRST_FLASH_SETTINGS_ADDRESS);
		SST25_WriteSR(SST25_BP1);*/
	}
	else{
		if(!validate_connection_params(&flash_settings)){
			memcpy(&flash_settings, &default_settings, sizeof(FLASH_SETTINGS_T));
			//PRINT_K("Cannot Validate Settings\n");
		}

	}
	print_settings();
	    /* if failed during SPI read load default settings back */
	//	SST25_Read(FIRST_FLASH_SETTINGS_ADDRESS, sizeof(flash_settings), (uint8_t *)&flash_settings);
		/* if failed during SPI read load default settings back */
	/*	ptr = (uint32_t *)&flash_settings;
		if(*ptr == 0xFFFFFFFF){
			PRINT_K("Loading defaults\n");
			memcpy(&flash_settings, &default_settings, sizeof(FLASH_SETTINGS_T));
		}*/
}
/********************************************************/
void Load_KmCounter()
{
	SST25_Read(KM_COUNTER_FLASH_ADDRESS, sizeof(u32_flashKmCounter), (uint8_t *)&u32_flashKmCounter);
	if(u32_flashKmCounter == 0xFFFFFFFF)
		u32_flashKmCounter = 0;
	PRINT_K("Km: ");
	PRINT_INT(u32_flashKmCounter);
	PRINT_K(CHAR_ENTER);
}
/*********************************************************/
void Load_BlockageSetting()
{
	if(!flash_settings.blockage_status_in_ram){
		if(flash_settings.engine_blockage)
			Set_BlockageStatus(BLOCKAGE_REQUESTED);
		else
			Set_BlockageStatus(BLOCKAGE_NOT_EXIST);
	}
}
/********************************************************/
/*void Init_EchoInfo()
{
	memset(&echo_packet, 0 ,sizeof(echo_packet));
}*/
/*********************************************************/
void BeginEchoPacket(char *pBuf, COMMAND_TYPE_T command)
{
	GSM_INFO_T gsm_info;
	char temp[4];

	Get_GsmInfo(&gsm_info);
	strcpy(pBuf, "@SET;");
	strcat(pBuf,gsm_info.imei_no);
	strcat(pBuf, CONFIG_PARAM_SEPERATOR);
	itoa((int)command, temp, 10);
	strcat(pBuf, temp);
	//strcat(pBuf, CONFIG_PARAM_SEPERATOR);
}
/*********************************************************/
void BeginGetResponse(char *pBuf, COMMAND_TYPE_T command)
{
	GSM_INFO_T gsm_info;

	Get_GsmInfo(&gsm_info);
	strcpy(pBuf, "@GET;INFO;");
	strcat(pBuf,gsm_info.imei_no);
	strcat(pBuf,"70");
	strcat(pBuf,";");
	strcat(pBuf, DEVICE_MODEL);
}
/**********************************************************/
void CloseEchoPacket(char *pBuf)
{
	strcat(pBuf, "!");
}
/***********************************************************/
void AddStringToEchoPacket(char *pBuf, char *strToAdd)
{
	strcat(pBuf, CONFIG_PARAM_SEPERATOR);
	strcat(pBuf, strToAdd);
}
/**************************************************************************************/
int32_t PrepareBlockageEchoPacket(char *buffer)
{
	BLOCKAGE_INFO_T blockage_info;

	Get_BlockageInfo(&blockage_info);

	BeginEchoPacket(buffer, ENGINE_BLOCKAGE_SETTING);
	AddStringToEchoPacket(buffer, "2");
	AddStringToEchoPacket(buffer, blockage_info.transactionID);
	CloseEchoPacket(buffer);
	return strlen(buffer);
}
/***********************************************************/
bool IsPasswordCorrect(char *pswd)
{
	if((strcmp(pswd, flash_settings.password) == 0) || (strcmp(pswd, ADMIN_PASSWORD) == 0))
		return TRUE;
	else
		return FALSE;
}
/*************************************************************/
void Get_UserSettings(FLASH_SETTINGS_T *user_settings)
{
	*user_settings = flash_settings;
}
/****************************************************************/
bool UpdateKmCounter(uint32_t u32_kmValue)
{
	uint32_t u32_totalKmWritten;

	//u32_totalKm = u32_flashKmCounter + u32_kmValue;

	//PRINT_K("Update KM:");
	SST25_WriteSR(0x00);
	SST25_WriteArray((uint8_t *)&u32_kmValue, sizeof(u32_kmValue), KM_COUNTER_FLASH_ADDRESS);
	SST25_WriteSR(SST25_BP1);
	SST25_Read(KM_COUNTER_FLASH_ADDRESS, sizeof(u32_totalKmWritten), (uint8_t *)&u32_totalKmWritten);

	/*uitoa(u32_totalKmWritten, buffer, 10);
	PRINT_K(buffer);*/
	if(u32_totalKmWritten == u32_kmValue){
	//	PRINT_K("Done\n");
		return true;
	}
	else{
	//	PRINT_K("Fail\n");
		return false;
	}
}
/****************************************************************/
void UpdateGpsBaudRate(uint32_t gps_baud_rate)
{
	flash_settings.gps_baud_rate = gps_baud_rate;
	Update_Settings();
}
/*************************************************************************/
void Get_UpdateServerParameters(TCP_CONNECTION_INFO_T *updateServerInfo_t)
{
	*updateServerInfo_t = firmwareUpdateServerInfo_t;
}
/**********************************************************************************/
COMMAND_RESULT_T ProcessReceivedData(char *dataBuffer, COMMAND_RESPONSE_T *response, COMMAND_SOURCE_T cmdSource)
{
	COMMAND_RESULT_T result = REPLY_DO_NOT_SEND;
	char  pswdBuf[MAX_SET_COMMAND_PSWD_LENGTH + 1];
	char *pPassword, *p_pswdStart;
	char *p_dataStart, *p_temp;

	p_dataStart = strstr(dataBuffer, TRIO_CONFIG_WORD);
	if(p_dataStart != NULL){
		p_temp = strchr(p_dataStart, '\r');
		if(p_temp != NULL)
			*p_temp = '\0';
		   	 p_pswdStart = strchr(p_dataStart ,':');
			 if(p_pswdStart != NULL) {  /* password available */
				pPassword = strtok(p_pswdStart, CONFIG_PARAM_SEPERATOR);
				memcpy(pswdBuf, pPassword + 1, MAX_SET_COMMAND_PSWD_LENGTH);
				if(!IsPasswordCorrect(pswdBuf)){
					return result;
				}
				p_dataStart = strlen(pswdBuf) + pPassword + 2;  /* points to data*/
			 }
			 else if(strlen(flash_settings.password) > 0){
				 return result;
			 }
			 else
				 p_dataStart += sizeof(TRIO_CONFIG_WORD);
			/* PRINT_K(p_dataStart);*/
		result = ParseConfigurationString(p_dataStart, response, cmdSource);
	}
	else{
		p_dataStart = strstr(dataBuffer, TRIO_GET_COMMAND);
	/*	PRINT_K(p_dataStart);*/
		if(p_dataStart != NULL){
			p_dataStart = &dataBuffer[sizeof(TRIO_GET_COMMAND)];
			result = ParseGetCommand(p_dataStart, response, cmdSource);
			/*PRINT_K(p_dataStart);*/
		}
	}
	return result;
}
/***********************************************************************/
uint16_t Get_MaxSpeedLimit()
{
	return flash_settings.u16_speedLimit;
}
/***********************************************************************/
uint16_t Get_MaxSpeedViolationDuration()
{
	return flash_settings.u16_speedLimitViolationDuration;
}
/************************************************************************/
uint32_t Get_MaxStopTime()
{
	return flash_settings.u32_maxStopTime;
}
/************************************************************************/
uint32_t Get_FlashKmValue()
{
	return u32_flashKmCounter;
}
/************************************************************************/
bool isRoamingActivated()
{
	return flash_settings.roaming_activation;
}
/************************************************************************/
bool validate_connection_params(FLASH_SETTINGS_T *settings)
{
	int i;
	bool found = FALSE;

	if(!isalnum(settings->flash_apn[0]))
		return FALSE;

	for(i = 0; i < MAX_SERVER_ADDR_LEN; i++){
		if(settings->server_ip[i] == '.'){
			found = TRUE;
			break;
		}
	}
	if(!found)
		return FALSE;

	if(atoi(settings->server_port) == 0)
		return FALSE;

	return TRUE;

}
/************************************************************************/
void write_default_settings()
{
	uint32_t u32_initialValues = 0;
//	uint32_t u32_logWriteStartAddress = 0xFFFFFFF0;// (PROTECTED_BLOCK_START_ADDRESS - 2* SST25_SECTOR_SIZE);
//	uint32_t u32_logReadStartAddress = 0xFFFFEFFE;
	uint32_t chksum;

    PRINT_K("\nWriting defaults\n");
	SST25_WriteSR(0x00);
	SST25_WriteArray((uint8_t *)&u32_initialValues,  4, KM_COUNTER_FLASH_ADDRESS);      /* ED000 */
	SST25_WriteArray((uint8_t *)&u32_initialValues , 4, OFFLINE_DATA_WRITE_ADDRESS);
	SST25_WriteArray((uint8_t *)&u32_initialValues , 4, OFFLINE_DATA_READ_ADDRESS);

	memcpy(&flash_settings, &default_settings, sizeof(FLASH_SETTINGS_T));
	chksum = calc_chksum((uint8_t *)&flash_settings, sizeof(flash_settings) -sizeof(flash_settings.chksum));
	flash_settings.chksum = chksum;

	SST25_WriteArray((uint8_t *)&flash_settings, sizeof(FLASH_SETTINGS_T), FIRST_FLASH_SETTINGS_ADDRESS);

	SST25_WriteSR(SST25_BP1);


}
/************************************************************************/
void print_settings()
{
	PRINT_LN(flash_settings.flash_apn);
	PRINT_LN(flash_settings.flash_apnpassword);
	PRINT_LN(flash_settings.flash_apnusername);
	PRINT_LN(flash_settings.device_id);
	PRINT_LN(flash_settings.server_ip);
	PRINT_LN(flash_settings.server_port);
	PRINT_LN(flash_settings.password);

	PRINT_INT(flash_settings.blockage_status_in_ram);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.roaming_activation);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.sms_activation);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.engine_blockage);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.u16_speedLimit);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.u16_speedLimitViolationDuration);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.u32_maxStopTime);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.msg_period_not_roaming_ignited);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.msg_period_not_roaming_not_ignited);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.msg_period_roaming_ignited);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.msg_period_roaming_not_ignited);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.u32_maxIdleTime);
	PRINT_K(CHAR_ENTER);

	PRINT_INT(flash_settings.gps_baud_rate);
	Print_Val("\nChecksum:", flash_settings.chksum);
}
