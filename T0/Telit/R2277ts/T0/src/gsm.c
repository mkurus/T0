#include "board.h"
#include "bsp.h"
#include "timer.h"
#include "messages.h"
#include "gsm.h"
#include "at_commands.h"
#include "bootloader.h"
#include "gps.h"
#include "settings.h"
#include "status.h"
#include "chip.h"
#include "spi.h"
#include "utils.h"
#include "xmodem1k.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GSM_UART         	 	 LPC_USART1
#define GSM_BAUD_RATE   		 115200
#define GSM_IRQ_SELECTION 	 	 UART1_IRQn
#define GSM_UART_ISR_HANDLER 	 UART1_IRQHandler



/* Transmit and receive ring buffer sizes */
#define GSM_UART_RRB_SIZE 512	/* Receive */
#define GSM_UART_SRB_SIZE 256 	/* Send */

#define MAX_TCP_RECEIVE_SIZE     256

#define HEARTBEAT_INTERVAL                   (1  * SECOND)
#define OFFLINE_DATA_SEND_INTERVAL           (10 * SECOND)
#define MODULE_STATUS_CHECK_INTERVAL         (3  * SECOND)
#define AT_RESPONSE_TIMEOUT                  (1  * SECOND)
#define GPRS_ATTACH_TIMEOUT                  (1  * MINUTE)
#define SERVER_CONNECTION_TIMEOUT            (30 * SECOND)
#define KEEP_ALIVE_TIMEOUT                   (5  * MINUTE)
#define GSM_REGISTRATION_TIMEOUT             (2  * MINUTE)

#define MAX_SMS_LENGTH                       160

#define MAX_NUMBER_OF_NONACKED_BYTES         500
#define AT_RESPONSE_BUFFER_SIZE              256

#define MAX_NUMBER_OF_FAIL_COUNT             5
#define MAX_NUMBER_OF_FAILS_FOR_MCU_RESET    5

#define SERVER_CONNECTION_RETRIES             5

void Gsm_AssertModuleResetLine(void);
void Gsm_DeassertModuleResetLine(void);

void DefinePDPContext(char *apn);
void ActivatePDPContext(char *apn, char *username, char *password);
void Send_ATCommand(const char *,char *);
void CloseSocketConnection();
void GSM_HardReset();
void Initialize_Timers();
uint16_t ProcessATResponse(char *, uint16_t length, uint16_t timeout, void (*callback)());
bool InsertImeiToOfflineData(char *buffer);
void Get_GsmRegistrationStatus(char * buffer);    /* get registration status  */
void Get_BatteryVoltage(char *buffer);
void Get_GsmSignalPower(char *buffer);           /* get CSQ value */
COMMAND_RESULT_T Get_SMS(char *buffer, COMMAND_RESPONSE_T *commandResponse, char *smsOrigAddr);
void Get_SocketStats(char *buffer);
void Get_SocketStatus(char *buffer);
int Get_Value(char *buffer, int param_no);
uint8_t GetSMSMessageIndex(char *);
uint8_t GetSMSContentLength(char *buffer);

#ifdef	T0_QUECTEL_M66
void SetIpAddressMode(bool b_alpha);
void SetForegroundContext(uint8_t context_num);
void SetRecvIndicatorMode(uint8_t indi_mode);
uint16_t Get_QuectelTCPPacket(char *buffer);
#endif
bool Extract_SmsOrigAddr(char * buffer, char *smsOrigAddrBuf);
void Start_SendingData(uint16_t i_msglen);
bool Restart_Modem();
bool ConnectToServer(uint8_t u8_retryCount);
bool ActivateGPRS(uint8_t u8_retryCount);
TIMER_INFO_T SEND_OFFLINE_DATA_TIMER;
TIMER_INFO_T SEND_PERIODIC_DATA_TIMER;
TIMER_INFO_T KEEP_ALIVE_TIMER;
TIMER_INFO_T SMS_CHECK_TIMER;
TIMER_INFO_T SEND_FAIL_TIMER;
TIMER_INFO_T HEARTBEAT_TIMER;


COMMAND_RESULT_T CheckModemStatus();

bool ModuleInitialize();
bool ConnectingToServer();
bool AttachToGprs();

void Init_UpdateInfo();
void Update_GsmRegistrationStatus(int u8_indValue);
void SendConnectionCommand(char *p_tcpPort, char *p_serverIp);
void Set_TcpConnectionParameters(char *ipAddress, char *tcpPort);

int ConnectedToServer();
/* Transmit and receive buffers */
static uint8_t gsm_rxbuff[GSM_UART_RRB_SIZE];
static uint8_t gsm_txbuff[GSM_UART_SRB_SIZE];
/* Transmit and receive ring buffers */
STATIC RINGBUFF_T gsm_txring, gsm_rxring;


TCP_CONNECTION_INFO_T tcpConnectionInfo_t;

GSM_INFO_T gsm_info;

static char messageBuffer[MAX_T_MESSAGE_SIZE];
static int32_t i_tempNonAckedBytes = 0;
static uint8_t u8_gsmModemFailCount = 0;
static uint8_t u8_counterMcuReset = 0;
static GSM_MODULE_STATE t_gsmState;
extern bool requestConnectToUpdateServer;
void Trio_Init_GSM_UART()
{
	Chip_UART_Init(GSM_UART);
	Chip_UART_ConfigData(GSM_UART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(GSM_UART, GSM_BAUD_RATE);
	Chip_UART_Enable(GSM_UART);
	Chip_UART_TXEnable(GSM_UART);

	/* Before using the ring buffers, initialize them using the ring
	   buffer init function */
/*	memset(gsm_rxbuff, 0, GSM_UART_RRB_SIZE);
	memset(gsm_txbuff, 0, GSM_UART_SRB_SIZE);*/
	RingBuffer_Init(&gsm_rxring, gsm_rxbuff, 1, GSM_UART_RRB_SIZE);
	RingBuffer_Init(&gsm_txring, gsm_txbuff, 1, GSM_UART_SRB_SIZE);

	Chip_UART_IntEnable(GSM_UART, UART_INTEN_RXRDY);
	NVIC_EnableIRQ(GSM_IRQ_SELECTION);

}
/*************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) GSM_UART_ISR_HANDLER(void)
{
	Chip_UART_IRQRBHandler(GSM_UART, &gsm_rxring, &gsm_txring);
}
/**************************************************************/
void Init_GSM_UART_PinMux(void)
{
	/* Enable the clock to the Switch Matrix */
	//Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

//	Chip_Clock_SetUARTClockDiv(1);	/* divided by 1 */

/*	Chip_SWM_DisableFixedPin(SWM_FIXED_ADC8);
	Chip_SWM_DisableFixedPin(SWM_FIXED_I2C0_SDA);*/

	Chip_SWM_MovablePinAssign(SWM_U1_TXD_O, 18);
	Chip_SWM_MovablePinAssign(SWM_U1_RXD_I, 11);

	/* Disable the clock to the Switch Matrix to save power */
	//Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}
/**************************************************************/
void ExecuteMainLoop()
{

	int i_msgLen;
	int result = 0;
#ifdef  T0_TELIT_GL865
	GSM_PowerOn();
	GSM_HardReset();
#elif   defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
	GSM_PowerOff();
	GSM_PowerOn();
	GSM_HardReset();
#endif

	//Set_Timer(&HEARTBEAT_TIMER, HEARTBEAT_INTERVAL);
	Set_Timer(&SEND_FAIL_TIMER, SEND_FAIL_TIMEOUT);
//	PRINT_K("\r\n");
	PRINT_K(VERSION);
	PRINT_K(CHAR_ENTER);
while(1){

		if(mn_timer_expired(&SEND_FAIL_TIMER))
			break;

		if(ifKmRecordLimitExceeded()){
			GPS_POSITION_DATA_T position_info;
			Get_PositionInfo(&position_info);
			UpdateKmCounter(Get_FlashKmValue() + position_info.distance/1000);
		}

		onIdle();
		Chip_WWDT_Feed(LPC_WWDT);

		switch(t_gsmState)
		{
			case MODULE_RESET_STATE:
			if(Restart_Modem() == TRUE){
			//	gsm_led_blink();
				u8_gsmModemFailCount = 0;
				u8_counterMcuReset = 0;
				t_gsmState = MODULE_INITIALIZING_STATE;
			}
			else{
				u8_counterMcuReset++;
				if(u8_counterMcuReset == MAX_NUMBER_OF_FAILS_FOR_MCU_RESET)
					NVIC_SystemReset();
				else{
					if(u8_gsmModemFailCount++ == MAX_NUMBER_OF_FAIL_COUNT){
						u8_gsmModemFailCount = 0;
						GSM_HardReset();
					}
				}
			}
			break;
			/******/
			case MODULE_INITIALIZING_STATE:
			if(ModuleInitialize() == TRUE){
				u8_gsmModemFailCount = 0;
				CheckModemStatus();
				if(gsm_info.b_roaming == TRUE){   /* check roaming status*/
					if(isRoamingActivated()){     /*  roaming allowed ? */
						PRINT_K("Roaming allowed\n");
						t_gsmState = MODULE_ATTACHING_TO_GPRS_STATE;
					}
					else{
						PRINT_K("Roaming not-allowed\n");
						Set_Timer(&SMS_CHECK_TIMER, MODULE_STATUS_CHECK_INTERVAL);
						t_gsmState = MODULE_WAIT_IN_ROAMING_STATE;
					}
				}
				else
					t_gsmState = MODULE_ATTACHING_TO_GPRS_STATE;
			}
			else{
				if(u8_gsmModemFailCount++ == MAX_NUMBER_OF_FAIL_COUNT * 5){
					u8_gsmModemFailCount = 0;
					t_gsmState = MODULE_RESET_STATE;
				}
			}
			break;
			/*****/
			case MODULE_WAIT_IN_ROAMING_STATE:
			if(mn_timer_expired(&SMS_CHECK_TIMER)){
				PRINT_K("Periodic check in roaming\n");
				CheckModemStatus();
				if(gsm_info.b_roaming){
					if(isRoamingActivated())
						t_gsmState = MODULE_ATTACHING_TO_GPRS_STATE;
					else
						Set_Timer(&SMS_CHECK_TIMER, MODULE_STATUS_CHECK_INTERVAL);
				}
				else{  /* not roaming */
					if(gsm_info.b_gsmRegistered)
						t_gsmState = MODULE_ATTACHING_TO_GPRS_STATE;
					else
						t_gsmState = MODULE_RESET_STATE;
				}
			}
			break;
			/*****/
			case MODULE_ATTACHING_TO_GPRS_STATE:
			if(AttachToGprs() == TRUE){
				PRINT_K("Attached to GPRS\n");
				u8_gsmModemFailCount = 0;
				FLASH_SETTINGS_T user_settings;
				Get_UserSettings(&user_settings);
				if(requestConnectToUpdateServer){
					requestConnectToUpdateServer = FALSE;
					if(ConnectToServer(SERVER_CONNECTION_RETRIES) == TRUE)
						t_gsmState = MODULE_CONNECTED_TO_UPDATE_SERVER_STATE;
					else{
						Set_ServerConnectionParameters(user_settings.server_ip,
													   user_settings.server_port);
						t_gsmState = MODULE_CONNECTING_TO_SERVER_STATE;
					}
				}
				else{
					Set_ServerConnectionParameters(user_settings.server_ip,
										       user_settings.server_port);
					t_gsmState = MODULE_CONNECTING_TO_SERVER_STATE;
				}
			}
			else{
				if(u8_gsmModemFailCount++ == MAX_NUMBER_OF_FAIL_COUNT){
					u8_gsmModemFailCount = 0;
					t_gsmState = MODULE_RESET_STATE;
				}
			}
			break;

			case MODULE_CONNECTING_TO_SERVER_STATE:
			if(ConnectingToServer() == TRUE){
				gsm_set_led_status(true);
			//	PRINT_K("Sending ST...");
				i_msgLen = Trio_PrepareSTMessage(messageBuffer);
				Send_MessageToServer(messageBuffer, i_msgLen, onIdle);
				i_msgLen = Trio_PrepareTMessage(messageBuffer, TRUE);
				if(Send_MessageToServer(messageBuffer, i_msgLen, onIdle)){
					PRINT_K("Done\n");
					gsm_info.b_socketActive = TRUE;
					u8_gsmModemFailCount = 0;
					Initialize_Timers();
					t_gsmState = MODULE_CONNECTED_TO_SERVER_STATE;
				}
				else{
					if(u8_gsmModemFailCount++ == MAX_NUMBER_OF_FAIL_COUNT){
						u8_gsmModemFailCount = 0;
						t_gsmState = MODULE_RESET_STATE;
						gsm_set_led_status(false);
					}
				}
			}
			else if(requestConnectToUpdateServer){
				requestConnectToUpdateServer = FALSE;
				if(ConnectToServer(SERVER_CONNECTION_RETRIES) == TRUE)
					t_gsmState = MODULE_CONNECTED_TO_UPDATE_SERVER_STATE;
				else
					t_gsmState = MODULE_RESET_STATE;
			}
			if(u8_gsmModemFailCount++ == MAX_NUMBER_OF_FAIL_COUNT){
				u8_gsmModemFailCount = 0;
				t_gsmState = MODULE_RESET_STATE;
			}
			break;

			case MODULE_CONNECTED_TO_SERVER_STATE:
			result = ConnectedToServer();
			if(result == -1){
				/*itoa(u8_gsmModemFailCount, buffer, 10);
				PRINT_K(buffer);*/
				if(u8_gsmModemFailCount++ == MAX_NUMBER_OF_FAIL_COUNT){
					u8_gsmModemFailCount = 0;
					gsm_info.b_socketActive = FALSE;
					t_gsmState = MODULE_RESET_STATE;
				}
				else{
					Initialize_Timers();
					PRINT_K("\nCONNECTED TO SERVER RETURNED ERROR\n");
					gsm_info.gsm_stats.u16_totalBytesNonAcked1 = gsm_info.gsm_stats.u16_totalBytesNonAcked2;
					i_tempNonAckedBytes = 0;
				}
			}
			else if(result == 1)
				u8_gsmModemFailCount = 0;
			else if(result == 4)
				t_gsmState = MODULE_CONNECTED_TO_UPDATE_SERVER_STATE;

			/* if roaming is not allowed drop connection*/
			if(gsm_info.b_roaming == TRUE){
				if(!isRoamingActivated()) {
					u8_gsmModemFailCount = 0;
					t_gsmState = MODULE_RESET_STATE;
				}
			}
			break;

			case MODULE_CONNECTED_TO_UPDATE_SERVER_STATE:
			/* GPS UART and firmware update routines use the same buffer. First disable
			*  the GPS interface to prevent intervention of GPS data*/
			Disable_GPS();
			Init_UpdateInfo();
		    XModem1K_Client(gsm_info.imei_no );
		    NVIC_SystemReset();
			break;
		}
	}
}
/****************************************************************************/
int ConnectedToServer()
{
	COMMAND_RESULT_T result;
//	EVENT_INFO_T event_info;
	COMMAND_RESPONSE_T commandResponse;
	BLOCKAGE_INFO_T blockage_info;
	int32_t i_msgLen = 0;
	int success = 0;


			Get_BlockageInfo(&blockage_info);
			if(blockage_info.blockageStatus== BLOCKAGE_ACTIVATED &&
			   blockage_info.cmdSource == COMMAND_SOURCE_TCP){
				i_msgLen = PrepareBlockageEchoPacket(messageBuffer);
				if(Send_MessageToServer(messageBuffer, i_msgLen, onIdle) == FALSE)
					return -1;
				Set_BlockageStatus(BLOCKAGE_NOT_EXIST);
			}


			if(Get_EventStatus()){
			//	PRINT_K("Send Event Info\n");
				i_msgLen = Trio_PrepareTMessage(messageBuffer, TRUE);
				if(Send_MessageToServer(messageBuffer, i_msgLen, onIdle) == FALSE)
					return -1;
				else{
					//PRINT_K("\r\nAlarm info sent to server\r\n");
					Set_Timer(&SEND_FAIL_TIMER, SEND_FAIL_TIMEOUT);
					Set_Timer(&KEEP_ALIVE_TIMER, KEEP_ALIVE_TIMEOUT);
					success = 1;
				}
			}
			if(mn_timer_expired(&SEND_PERIODIC_DATA_TIMER)) {
					PRINT_K("\nSend periodic data\n");
					i_msgLen = Trio_PrepareTMessage(messageBuffer, TRUE);
					if(Send_MessageToServer(messageBuffer, i_msgLen, onIdle) == FALSE)
						return -1;
					else{
						Set_Timer(&SEND_PERIODIC_DATA_TIMER, Get_DataSendPeriod());
						Set_Timer(&KEEP_ALIVE_TIMER, KEEP_ALIVE_TIMEOUT);
						Set_Timer(&SEND_FAIL_TIMER, SEND_FAIL_TIMEOUT);
						success = 1;
					}
			}
			if(mn_timer_expired(&KEEP_ALIVE_TIMER)){
				PRINT_K("\nSend Keep-Alive\n");
				i_msgLen = Trio_PreparePingMessage(messageBuffer, gsm_info.imei_no);
				if(Send_MessageToServer(messageBuffer, i_msgLen, onIdle) == FALSE)
					return -1;
				else{
					Set_Timer(&KEEP_ALIVE_TIMER, KEEP_ALIVE_TIMEOUT);
					Set_Timer(&SEND_FAIL_TIMER, SEND_FAIL_TIMEOUT);
					success = 1;
				}
			}
			if(mn_timer_expired(&SEND_OFFLINE_DATA_TIMER)){
					i_msgLen = Get_OffLineDataFromFlash(messageBuffer);
					if(i_msgLen > 0){
						PRINT_K("\nSend Log\n");
						if(InsertImeiToOfflineData(messageBuffer)){
							if(Send_MessageToServer(messageBuffer, strlen(messageBuffer), onIdle) == FALSE)
								return -1;
							else{
								Set_Timer(&KEEP_ALIVE_TIMER, KEEP_ALIVE_TIMEOUT);
								Set_Timer(&SEND_FAIL_TIMER, SEND_FAIL_TIMEOUT);
								success = 1;
							}
						}
					}
				Set_Timer(&SEND_OFFLINE_DATA_TIMER, OFFLINE_DATA_SEND_INTERVAL);
			}

			if(mn_timer_expired(&SMS_CHECK_TIMER)){
				PRINT_K("\nPeriodic Check\n");
				if((CheckModemStatus() == REPLY_CONNECT_TO_UPDATE_SERVER) || (requestConnectToUpdateServer == TRUE)){
					requestConnectToUpdateServer = FALSE;
					Delay(100, NULL);
					Gsm_CloseSocket();
					if(ConnectToServer(SERVER_CONNECTION_RETRIES) == TRUE)
						return 4;
					else return -1;
				}
#if  defined(T0_TELIT_GL865)
				if(gsm_info.gsm_stats.u16_totalBytesBuffered > 0)
#endif
				{
					if(Get_GsmTcpBuffer(messageBuffer, MAX_T_MESSAGE_SIZE, onIdle) > 0){
						memset(&commandResponse, 0, sizeof(commandResponse));
						result = ProcessReceivedData(messageBuffer, &commandResponse, COMMAND_SOURCE_TCP);
						bool sendOk = TRUE;
						if(result == REPLY_ECHO_PACKET){
							PRINT_K(commandResponse.buffer);
							sendOk = Send_MessageToServer(commandResponse.buffer,
														  strlen(commandResponse.buffer),
														  onIdle);
						}
						else if(result == REPLY_CONNECT_TO_UPDATE_SERVER){
							Gsm_CloseSocket();
							if(ConnectToServer(5) == TRUE)
								return 4;
						}
						if(sendOk == FALSE)
							return -1;
						else{
							Set_Timer(&SEND_FAIL_TIMER, SEND_FAIL_TIMEOUT);
							Set_Timer(&KEEP_ALIVE_TIMER, KEEP_ALIVE_TIMEOUT);
							if(commandResponse.b_needToReset){
								PRINT_K("\nNeed to reset\n");
								NVIC_SystemReset();
							}
							success = 1;
						}
					}
				}
				if((i_tempNonAckedBytes > MAX_NUMBER_OF_NONACKED_BYTES) ||
					(gsm_info.b_gsmRegistered == FALSE)                 ||
					(!gsm_info.b_socketActive))
						return -1;

				Set_Timer(&SMS_CHECK_TIMER, MODULE_STATUS_CHECK_INTERVAL);
			}
			return success;
}
/*****************************************************/
bool InsertImeiToOfflineData(char *buffer)
{
	char tempBuffer[MAX_T_MESSAGE_SIZE];
	char *p_msgHeader, *p_msgFooter;

	memset(tempBuffer, 0, MAX_T_MESSAGE_SIZE);

	/* check if we have a valid T message header */
	p_msgHeader = strstr(buffer, T_MESSAGE_HEADER);
	if(p_msgHeader == NULL)
		return FALSE;

	/* check if we have a valid T message footer */
	p_msgFooter = strstr(buffer, T_MESSAGE_FOOTER);
	if(p_msgFooter == NULL)
		return FALSE;

	/* check if we have a valid IMEI */
	if(strlen(gsm_info.imei_no) != IMEI_LEN)
		return FALSE;

	/* start copying message to buffer*/
	strcat(tempBuffer, T_MESSAGE_HEADER);
	*p_msgFooter ='\0';

	strcat(tempBuffer, gsm_info.imei_no);
	strcat(tempBuffer, p_msgHeader + sizeof(T_MESSAGE_HEADER) - 1);

	Trio_AddTMessageExtensionSeperator(tempBuffer);
	Trio_PrepareTMessageExtension(tempBuffer, OFFLINE_DATA);
	Trio_EndTMessage(tempBuffer);

	strcpy(buffer, tempBuffer);

	return TRUE;
}

/*********************************************************************/
void ReloadPeriodicDataSendTimer()
{
	Set_Timer(&SEND_PERIODIC_DATA_TIMER, Get_DataSendPeriod());
}
/**********************************************************************/
bool ConnectingToServer()
{
	uint8_t result = true;

#ifdef T0_TELIT_GL865
	    char buffer[128];
		Send_ATCommand(Command_SCFG, "");
		ProcessATResponse(buffer, sizeof(buffer), AT_RESPONSE_TIMEOUT, onIdle);
		if(strstr(buffer, RespOK) == NULL)
			return false;

		/*********/

		Send_ATCommand(Command_SCFGEXT, "");
		ProcessATResponse(buffer, 32, AT_RESPONSE_TIMEOUT, onIdle);
		if(strstr(buffer, RespOK) == NULL)
			return false;

/*#elif	defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
		Send_ATCommand(Command_SetRecvHdr, "Setting Receive Header...");
		ProcessATResponse(buffer, 32, AT_RESPONSE_TIMEOUT, onIdle);
		if(strstr(buffer, RespOK) == NULL)
			return false;

#elif  defined(T0_SIMCOM_SIM800C)
		Send_ATCommand(Command_SetRxMode, "Setting Receive Mode...");
		ProcessATResponse(buffer, 32, AT_RESPONSE_TIMEOUT, onIdle);
		if(strstr(buffer, RespOK) == NULL)
			return false;*/
#endif

		result = ConnectToServer(SERVER_CONNECTION_RETRIES);
		return result;
}
/*********************************************************************/
bool ConnectToServer(uint8_t u8_retryCount)
{

	char respBuffer[64];

	while(u8_retryCount > 0){
		CheckModemStatus();
		Delay(500, onIdle);
		SendConnectionCommand(tcpConnectionInfo_t.tcpServerInfo_t.server_ip,
							  tcpConnectionInfo_t.tcpServerInfo_t.server_port);
		ProcessATResponse(respBuffer, sizeof(respBuffer), 200, onIdle);
#ifdef T0_TELIT_GL865
		if(strstr(respBuffer, RespOK) != NULL)
#elif defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
		if(strstr(respBuffer,RespConnectOK) != NULL || strstr(respBuffer,RespAlrdyConnect) != NULL )
#endif
			return TRUE;
		else
			u8_retryCount--;
	}
	return FALSE;
}
/*****************************************************************************/
bool AttachToGprs()
{
	TIMER_INFO_T AT_RESPONSE_TIMER;
	FLASH_SETTINGS_T user_settings;
	char buffer[32];

	 Get_UserSettings(&user_settings);
#ifdef	T0_QUECTEL_M66
	      SetIpAddressMode(isalpha(user_settings.server_ip[0]));
	      SetForegroundContext(1);
	      SetRecvIndicatorMode(1);
#endif
#if   defined(T0_TELIT_GL865) || defined(T0_SIMCOM_SIM800C)
			/***********/
			DefinePDPContext(user_settings.flash_apn);
			ProcessATResponse(buffer, sizeof(buffer), AT_RESPONSE_TIMEOUT, onIdle);
			if(strstr(buffer, RespOK) == NULL)
				return false;
			PRINT_K("Activate PDP...");
#endif

			Set_Timer(&AT_RESPONSE_TIMER, GPRS_ATTACH_TIMEOUT);
			while(!mn_timer_expired(&AT_RESPONSE_TIMER)){
				ActivatePDPContext(user_settings.flash_apn,
								   user_settings.flash_apnusername,
							       user_settings.flash_apnpassword);

				ProcessATResponse(buffer, sizeof(buffer), GPRS_ATTACH_TIMEOUT / 6, onIdle);
#ifdef T0_TELIT_GL865
				if((strstr(buffer,"#SGACT") != NULL) || (strstr(buffer, RespOK) != NULL))
#elif defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
				if(strstr(buffer, RespOK) != NULL)
#endif
				{
					PRINT_K("Done\n");
#ifdef T0_SIMCOM_SIM800C
					break;
#else
					return TRUE;
#endif
				}
				CheckModemStatus();
				if(gsm_info.b_gsmRegistered == FALSE)
					return FALSE;
			}
#ifdef T0_SIMCOM_SIM800C
			PRINT_K("Connect to GPRS...");
			Set_Timer(&AT_RESPONSE_TIMER, GPRS_ATTACH_TIMEOUT);
			Send_ATCommand(Command_ActivateGPRS,"");
			while(!mn_timer_expired(&AT_RESPONSE_TIMER)){
				ProcessATResponse(buffer, sizeof(buffer), GPRS_ATTACH_TIMEOUT / 6, onIdle);
				if(strstr(buffer, RespOK) != NULL){
					PRINT_K("Done\n");

					Send_ATCommand(Command_GetLocalIP,"");
					ProcessATResponse(buffer, sizeof(buffer), 300, onIdle);

					return TRUE;
				}
				CheckModemStatus();
				if(gsm_info.b_gsmRegistered == FALSE)
					return FALSE;
			}
#endif
		return FALSE;
}
/*****************************************************************************/
/* Initialization state machine for GSM module. Returns true on success.                                            */
/*****************************************************************************/
bool ModuleInitialize()
{
	TIMER_INFO_T AT_RESPONSE_TIMER;
	char buffer[256];
	bool success = FALSE;

	gsm_info.b_gsmHealtStatus = FALSE;
			/****/
			Set_Timer(&AT_RESPONSE_TIMER, 1000);
			while(!mn_timer_expired(&AT_RESPONSE_TIMER)){
				Send_ATCommand(Command_AT, "Init. module\n");
				ProcessATResponse(buffer, sizeof(buffer), 100, onIdle);

				if(strstr(buffer, RespOK) != NULL)
					break;
				else
					return FALSE;
			}

			/***/
			Send_ATCommand(Command_ATE0, "Echo disable\n");
			ProcessATResponse(buffer, 192, 200, onIdle);
			if(strstr(buffer, RespOK) == NULL)
				return false;

			/*****/
		///	char tempBuf[] = "\r\n868455655123526\r\n+CPIN: READY\r\nOK\r\n";
			Send_ATCommand(Command_CGSN, "");
		//	memcpy(buffer, tempBuf, sizeof(tempBuf));
	    	ProcessATResponse(buffer, 128, AT_RESPONSE_TIMEOUT, onIdle);
			if(strstr(buffer, RespOK) == NULL)
				return false;

			success = ExtractIMEI(buffer,gsm_info.imei_no);
			PRINT_K("\nIMEI: ");
			PRINT_K(gsm_info.imei_no);
			PRINT_K("\n");
			if(!success)
				return false;

			/*****/
			Set_Timer(&AT_RESPONSE_TIMER, 1000);
			while(!mn_timer_expired(&AT_RESPONSE_TIMER)){
				Send_ATCommand(Command_CPIN,"\nRead PIN\n");
				ProcessATResponse(buffer, 64, AT_RESPONSE_TIMEOUT, onIdle);
				if(strstr(buffer, RespReady) != NULL){
					success = TRUE;
					break;
				}
				else
					success = FALSE;
			}
			if(!success)
				return FALSE;

			/*****/
			Send_ATCommand(Command_CIMI, "");
			ProcessATResponse(buffer, 64, AT_RESPONSE_TIMEOUT, onIdle);
			if(strstr(buffer, RespOK) == NULL)
				return false;
			memcpy(gsm_info.imsi_no, &buffer[2] ,IMSI_LEN);
			gsm_info.imsi_no[IMSI_LEN] = '\0';
			gsm_info.b_gsmHealtStatus = TRUE;

			PRINT_K("\nIMSI: ");
			PRINT_K(gsm_info.imsi_no);
			PRINT_K("\n");
			/******/

#ifdef T0_TELIT_GL865
			Set_Timer(&AT_RESPONSE_TIMER, 1000);
			while(!mn_timer_expired(&AT_RESPONSE_TIMER)){
				Send_ATCommand(Command_CMER, "");
				ProcessATResponse(buffer, 64, AT_RESPONSE_TIMEOUT, onIdle);
				if(strstr(buffer, RespOK) != NULL){
					success = TRUE;
					break;
				}
				else
					success = FALSE;
			}
			if(!success)
				return FALSE;
#endif
			/******/
			Set_Timer(&AT_RESPONSE_TIMER, GSM_REGISTRATION_TIMEOUT);
			while(!mn_timer_expired(&AT_RESPONSE_TIMER)){
				Send_ATCommand(Command_CREG, "\nWaiting GSM Registration...\n");
				ProcessATResponse(buffer, 64, AT_RESPONSE_TIMEOUT, onIdle);
				Get_GsmRegistrationStatus(buffer);
				if(gsm_info.b_gsmRegistered){
					PRINT_K("Registered\n");
					return true;
				}
			}
		return false;
}
/*************************************************************/
bool ExtractIMEI(char *buffer, char *imei)
{
	while(*buffer != '\0'){
		if(isdigit(*buffer)){
			memcpy(imei, buffer, IMEI_LEN);
			gsm_info.imei_no[IMEI_LEN] = '\0';
			return TRUE;
		}
		else
			buffer++;
	}
	return FALSE;
}
/*************************************************************/
#define   AT_RESPONSE_CHAR_TIMEOUT            10

uint16_t ProcessATResponse(char *buffer, uint16_t length, uint16_t timeout, void (*callbackFunc)())
{
	TIMER_INFO_T at_response_timer;
	TIMER_INFO_T charTimer;
	int readBytes = 0;
	char data;

	memset(buffer,0,length);

    Set_Timer(&at_response_timer, timeout);

    while(!mn_timer_expired(&at_response_timer)){
    	if(Chip_UART_ReadRB(GSM_UART, &gsm_rxring, &data, 1) > 0){
    		Set_Timer(&charTimer, AT_RESPONSE_CHAR_TIMEOUT);
    		buffer[readBytes++] = data;
    		if(readBytes == length){
    			return readBytes;
    		}
    	 }
    	else{
    		if((mn_timer_expired(&charTimer)) && ( readBytes > 0)){
    		//	PRINT_K(buffer);
    			return readBytes;
    		}
    	}
    	if(callbackFunc != NULL)
    		callbackFunc();
    }
    return 0;
}

/*****************************************************************************/
void Update_GsmRegistrationStatus(int i_indValue)
{
	gsm_info.b_gsmRegistered = FALSE;
	gsm_info.b_roaming = FALSE;

	if(i_indValue == 1)
		gsm_info.b_gsmRegistered = TRUE;

	else if(i_indValue == 5){
		gsm_info.b_gsmRegistered = TRUE;
		gsm_info.b_roaming = TRUE;
	}
}
/********************************************************************************/
void Set_GsmResetPinFunction(uint8_t pin_number)
{
	/* Set PIO0.10 as output.This pin controls the reset pin */
	/* of Telit GSM module*/

	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/* Enable GPIO function for I2C pins */
	/*Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO10, PIN_I2CMODE_GPIO);*/
	/* Disable the clock to the Switch Matrix to save power */
/*	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);*/

	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, pin_number);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, pin_number, true);
}

/******************************************************************/
void Get_GsmInfo(GSM_INFO_T *const gsm_info_buffer)
{
	*gsm_info_buffer = gsm_info;
	/*gsm_info_buffer->csq = gsm_params_t.csq;
	strcpy(gsm_info_buffer->imei_no, gsm_params_t.imei_no);
	strcpy(gsm_info_buffer->imsi_no, gsm_params_t.imsi_no);*/
}
/*******************************************************************/
void DeassertGsmResetPin(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GSM_RESET_PIN, false);
}

/*********************************************************************/
void Send_ATCommand(const char *command,  char *traceMessage)
{
	PRINT_K(traceMessage);
	PRINT_K(command);
	RingBuffer_Init(&gsm_rxring, gsm_rxbuff, 1, GSM_UART_RRB_SIZE);
	Chip_UART_SendRB(GSM_UART, &gsm_txring, command, strlen(command));
}
/**********************************************************************/
bool GetServerConnectionStatus()
{
	return gsm_info.b_socketActive;
}
/***********************************************************************/
void DefinePDPContext(char *apn)
{
	char cmdBuffer[256];

	memset(cmdBuffer,0, sizeof(cmdBuffer));
	strcat(cmdBuffer,Command_DefinePDP);
	strcat(cmdBuffer, apn);
	strcat(cmdBuffer,"\"\r");

	Send_ATCommand(cmdBuffer, "");
}
/***********************************************************************/
void ActivatePDPContext(char *apn, char *username, char *password)
{
	char cmdBuffer[256];

	memset(cmdBuffer,0, sizeof(cmdBuffer));
	strcat(cmdBuffer,Command_ActivatePDP);
#ifdef T0_TELIT_GL865
	strcat(cmdBuffer, username);
	strcat(cmdBuffer,"\",\"");
	strcat(cmdBuffer, password);
	strcat(cmdBuffer,"\"\r");
#elif defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
	strcat(cmdBuffer, apn);
	strcat(cmdBuffer,"\",\"");
	strcat(cmdBuffer, username);
	strcat(cmdBuffer,"\",\"");
	strcat(cmdBuffer, password);
	strcat(cmdBuffer,"\"\r");
#endif
	Send_ATCommand(cmdBuffer, "");
}
/*************************************************************************/
void SendConnectionCommand(char *p_serverIp, char *p_tcpPort)
{
	char cmdBuffer[256];

	memset(cmdBuffer,0, sizeof(cmdBuffer));

	strcat(cmdBuffer,Command_TCPConnect);
#ifdef T0_TELIT_GL865
	strcat(cmdBuffer, p_tcpPort);
	strcat(cmdBuffer,",\"");
	strcat(cmdBuffer, p_serverIp);
	strcat(cmdBuffer,"\",0,0,1\r");
#elif defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
	strcat(cmdBuffer, p_serverIp);
	strcat(cmdBuffer,"\",\"");
	strcat(cmdBuffer, p_tcpPort);
	strcat(cmdBuffer,"\"\r");
#endif
	Send_ATCommand(cmdBuffer, "\nConnecting...");
}
/*********************************************************************/
/* Returns the periodic data sent timeout based on the user settings */
/*********************************************************************/
TIMER_TICK_T Get_DataSendPeriod()
{
	FLASH_SETTINGS_T user_settings;
	Get_UserSettings(&user_settings);

//	if(Get_IgnitionStatus() && (!gsm_info.b_roaming))
	if(Get_IgnitionPinStatus() && (!gsm_info.b_roaming))
		return user_settings.msg_period_not_roaming_ignited *100;

	else if(!Get_IgnitionPinStatus() && (!gsm_info.b_roaming)){
		if(IsMoving())
			return user_settings.msg_period_not_roaming_ignited *100;
		else
			return user_settings.msg_period_not_roaming_not_ignited *100;
	}

	else if(Get_IgnitionPinStatus() && (gsm_info.b_roaming))
		return user_settings.msg_period_roaming_ignited *100;

	else if(!Get_IgnitionPinStatus() && (gsm_info.b_roaming)){
		if(IsMoving())
			return user_settings.msg_period_roaming_ignited *100;
		else
			return user_settings.msg_period_roaming_not_ignited *100;
	}

	return 500;
}
/****************************************************************************/
void Init_GsmInfo()
{
//	memset(&gsm_info,  0, sizeof(gsm_info));
	t_gsmState = MODULE_RESET_STATE;
	gsm_info.b_gsmHealtStatus = FALSE;
}
/*****************************************************************************/
void Set_ServerConnectionParameters(char *server_ip, char *server_port)
{
	strcpy(tcpConnectionInfo_t.tcpServerInfo_t.server_ip, server_ip);
	strcpy(tcpConnectionInfo_t.tcpServerInfo_t.server_port, server_port);

}
/*******************************************************************************/
void Delay(TIMER_TICK_T delay, void (*callbackFunc)())
{
	TIMER_INFO_T wait_timer;
	Set_Timer(&wait_timer, delay);
	while (!mn_timer_expired(&wait_timer)){
		if(callbackFunc != NULL)
			callbackFunc();
	}
}
/********************************************************************************/
void GSM_HardReset()
{
	PRINT_K("\nModule hard reset\n");
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GSM_RESET_PIN, true);
	Delay(HARD_RESET_LOW_TIME, onIdle);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GSM_RESET_PIN, false);
	Delay(WAIT_TIME_AFTER_RESET, onIdle);
}
/********************************************************************************/
void GSM_PowerOn()
{
	PRINT_K("\nPower-On Module\n");
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GSM_POWER_ON_OFF_PIN, true);
	Delay(10, onIdle);
}
/********************************************************************************/
void GSM_PowerOff()
{
	PRINT_K("\nPower-Off Module\n");
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GSM_POWER_ON_OFF_PIN, false);
	Delay(80, onIdle);
}
/****************************************************************/
void TogglePin(uint8_t pin_number, uint16_t duration)
{
	Delay(duration, onIdle);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, pin_number, TRUE);
	Delay(duration, onIdle);
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, pin_number, FALSE);
}
/**********************************************************************************/
bool Restart_Modem()
{
	char buffer[32];
	RingBuffer_Init(&gsm_rxring, gsm_rxbuff, 1, GSM_UART_RRB_SIZE);
	RingBuffer_Init(&gsm_txring, gsm_txbuff, 1, GSM_UART_SRB_SIZE);

/*	while(i < 10){
		Send_ATCommand(Command_AT, "Auto-bauding\r\n");
		ProcessATResponse(buffer,sizeof(buffer), 100, onIdle);
		i++;
	}*/

	Send_ATCommand(Command_RESET, "\nModule soft-reset\n");
	ProcessATResponse(buffer,sizeof(buffer), 500, onIdle);
	if(strstr(buffer, RespOK) == NULL)
		return false;

//	PRINT_K("\r\nModem soft-resetted\r\n");
	return true;
}
/************************************************************************/
bool Send_MessageToServer(char *pBuffer, uint16_t i_msgLen, void (*onIdleCallback)())
{
	char responseBuf[128];

	Start_SendingData(i_msgLen);
	ProcessATResponse(responseBuf, sizeof(responseBuf), 200, onIdleCallback);
	if(strchr(responseBuf, DATA_SEND_PROMPT) != NULL){
		Send_ATCommand(pBuffer,"");
		PRINT_K(pBuffer);
#ifdef T0_SIMCOM_SIM800C
		char dataEnd[2];
		dataEnd[0] = CTRL_Z;  /* SIMCOM modules require Ctrl-Z  to start TCP data transmission*/
		dataEnd[1] = '\0';
		Send_ATCommand(dataEnd, "");
#endif
		ProcessATResponse(responseBuf, sizeof(responseBuf), 200, onIdleCallback);
#ifdef T0_TELIT_GL865
		if(strstr(responseBuf, RespOK) == NULL)
#elif defined (T0_QUECTEL_M66) || defined (T0_SIMCOM_SIM800C)
		if(strstr(responseBuf, RespSendOK) == NULL)
#endif
			return false;
		else
			return true;
		}
		else
			return false;
}
/************************************************************/
#define	MAX_NUMBER_OF_SOCKET_RETRIES   5
bool Gsm_CloseSocket()
{
	uint8_t i =0;
	char buffer[32];

	while(i < MAX_NUMBER_OF_SOCKET_RETRIES){
		Send_ATCommand(Command_CloseSocket, "\nClosing socket...");
		ProcessATResponse(buffer, sizeof(buffer), 500, NULL);
		if(strstr(buffer, RespOK) != NULL){
			PRINT_K("Done\n");
			return TRUE;
		}
		i++;
	}
	return FALSE;
}
/******************************************************************************/
void Start_SendingData(uint16_t i_msgLen)
{
	char temp[8];

	char cmdBuffer[32];

	memset(cmdBuffer,0, sizeof(cmdBuffer));

	itoa(i_msgLen, temp, 10);

	strcat(cmdBuffer,Command_SendData);
	strcat(cmdBuffer,temp);
	strcat(cmdBuffer, "\r");
	Send_ATCommand(cmdBuffer,"");
}
/*******************************************************************************/
#ifdef	T0_QUECTEL_M66
void SetIpAddressMode(bool b_alpha)
{
	char cmdBuffer[32];
	memset(cmdBuffer,0, sizeof(cmdBuffer));
	strcat(cmdBuffer,Command_SetIpAddrMode);
	if(b_alpha)
		strcat(cmdBuffer,"1\r");
	else
		strcat(cmdBuffer,"0\r");
	Send_ATCommand(cmdBuffer,"");
}
/********************************************************************************/
void SetForegroundContext(uint8_t context_num)
{
	char cmdBuffer[32];
	char temp[4];

	itoa(context_num, temp, 10);
	memset(cmdBuffer,0, sizeof(cmdBuffer));
	strcat(cmdBuffer,Command_SetFgndContext);
	strcat(cmdBuffer,temp);
	strcat(cmdBuffer, "\r");
	Send_ATCommand(cmdBuffer,"");
}
/********************************************************************************/
void SetRecvIndicatorMode(uint8_t indi_mode)
{
	char cmdBuffer[32];
	char temp[4];

	itoa(indi_mode, temp, 10);
	memset(cmdBuffer,0, sizeof(cmdBuffer));
	strcat(cmdBuffer,Command_SetRecvIndiMode);
	strcat(cmdBuffer,temp);
	strcat(cmdBuffer, "\r");
	Send_ATCommand(cmdBuffer,"");
}

#endif
/*******************************************************************************/
void Initialize_Timers()
{
	Set_Timer(&KEEP_ALIVE_TIMER, KEEP_ALIVE_TIMEOUT);
	Set_Timer(&SEND_OFFLINE_DATA_TIMER, OFFLINE_DATA_SEND_INTERVAL);
	Set_Timer(&SEND_PERIODIC_DATA_TIMER, Get_DataSendPeriod());
	Set_Timer(&SMS_CHECK_TIMER, MODULE_STATUS_CHECK_INTERVAL);
}
/*********************************************************************************/
bool Send_SMS(char *destAddr, char *smsText, void (*onIdleCallback)())
{
	TIMER_INFO_T sms_sent_timer;
	char cmdBuffer[200];
	char ctrlStr[2] = {CTRL_Z, '\0'};

	memset(cmdBuffer,0, sizeof(cmdBuffer));
	strcat(cmdBuffer,Command_CMGS);
	strcat(cmdBuffer,destAddr);
/*
* Quectel module requires command in AT+CMGS="xxxxxxxxxx"\r format
* Telit module requires command   in AT+CMGS=xxxxxxxxxx\r   format
*/
#ifdef T0_QUECTEL_M66
	strcat(cmdBuffer,"\"");
#endif
	strcat(cmdBuffer, "\r");

	Send_ATCommand(cmdBuffer,"Sending SMS...\n");
	memset(cmdBuffer,0, sizeof(cmdBuffer));
	ProcessATResponse(cmdBuffer, sizeof(cmdBuffer), 200, onIdleCallback);

	if(strchr(cmdBuffer, DATA_SEND_PROMPT) != NULL){
		memset(cmdBuffer,0, sizeof(cmdBuffer));
		strcat(cmdBuffer,smsText);
		strcat(cmdBuffer,ctrlStr);
		Send_ATCommand(cmdBuffer,"");

		Set_Timer(&sms_sent_timer, SMS_SENT_RESPONSE_TIMEOUT);
		while(!mn_timer_expired(&sms_sent_timer)){
			memset(cmdBuffer,0, sizeof(cmdBuffer));
			ProcessATResponse(cmdBuffer, sizeof(cmdBuffer), 200, onIdleCallback);
			if(strstr(cmdBuffer, RespOK) != NULL){
				PRINT_K("SMS Sent\n");
				return TRUE;
			}
		}
		PRINT_K("Cannot Send SMS\n");
	}
	return FALSE;
}
/*********************************************************************************/
COMMAND_RESULT_T CheckModemStatus()
{
	COMMAND_RESULT_T result;
	COMMAND_RESPONSE_T commandResponse;
	char buffer[200];
	char smsOrigAddrBuf[32];

	Send_ATCommand(Command_CMGR,"");
	ProcessATResponse(buffer, sizeof(buffer), 200, onIdle);

	PRINT_K(buffer);
	Get_GsmRegistrationStatus(buffer);    /* get registration status  */
	Get_BatteryVoltage(buffer);
	Get_GsmSignalPower(buffer);           /* get CSQ value */

	memset(smsOrigAddrBuf,0, sizeof(smsOrigAddrBuf));
	result  = Get_SMS(buffer, &commandResponse, smsOrigAddrBuf);
	if(result == REPLY_ECHO_PACKET){
		PRINT_K(commandResponse.buffer);
		Send_SMS(smsOrigAddrBuf, commandResponse.buffer, onIdle);
		if(commandResponse.b_needToReset)
			NVIC_SystemReset();
	}

	Get_SocketStats(buffer);
	Get_SocketStatus(buffer);
	return result;

}
/*************************************************************************
 * Returns an AT response parameter specified by param_no                *
 *************************************************************************/
int Get_ValueByParamIndex(char *buffer, int param_no)
{
	char *p_temp;
	char *p_start;
	int i_commaCount =0;
	char tempBuffer[128];

	memset(tempBuffer, 0, sizeof(tempBuffer));
	memcpy(tempBuffer, buffer, sizeof(tempBuffer));

	p_start = strchr(tempBuffer,' ');
	if(p_start == NULL)
		return 0;
	p_temp = strtok(p_start,"\r,");
	while (p_temp != NULL){
		if(++i_commaCount == param_no)
			return atoi(p_temp);
		p_temp = strtok (NULL, "\r,");
	}
	return 0;
}
/********************************************************************/
void Get_GsmRegistrationStatus(char *buffer)
{
	char *p_temp;
	int reg;
	/* check registration status*/
	p_temp = strstr(buffer, RespCREG);
	if(p_temp != NULL){
		reg = Get_ValueByParamIndex(p_temp, 2);
		Update_GsmRegistrationStatus(reg);
	}
}
/********************************************************************/
void Get_BatteryVoltage(char *buffer)
{
	char *p_temp;

#if defined(T0_TELIT_GL865)
    p_temp = strstr(buffer, RespCBC);
    if(p_temp != NULL)
    	gsm_info.batteryLevel = Get_ValueByParamIndex(p_temp, 2) * 10;
#elif defined (T0_QUECTEL_M66) || defined (T0_SIMCOM_SIM800C)
    p_temp = strstr(buffer, RespCBC);
    if(p_temp != NULL)
       	gsm_info.batteryLevel = Get_ValueByParamIndex(p_temp, 3);
#endif
}
/*********************************************************************/
void Get_GsmSignalPower(char *buffer)
{
	char *p_temp;
    p_temp = strstr(buffer, RespCSQ);
	if(p_temp != NULL)
	    gsm_info.csq = Get_ValueByParamIndex(p_temp, 1);
}
/********************************************************************/

COMMAND_RESULT_T Get_SMS(char *buffer, COMMAND_RESPONSE_T *commandResponse, char *smsOrigAddrBuf)
{
	FLASH_SETTINGS_T user_settings;
	COMMAND_RESULT_T result  =  REPLY_DO_NOT_SEND;
	//COMMAND_RESPONSE_T commandResponse;
//	char smsOrigAddrBuf[64];
	char *p_temp;
	char *p_data;

//	memset(smsOrigAddrBuf,0, sizeof(smsOrigAddrBuf));
	Extract_SmsOrigAddr(buffer, smsOrigAddrBuf);
	PRINT_K(smsOrigAddrBuf);
	Get_UserSettings(&user_settings);
	memset(commandResponse, 0, sizeof(COMMAND_RESPONSE_T));
	/* check SMS*/
	p_temp = strstr(buffer, RespCMGR);
	if(p_temp != NULL){
		p_data = strstr(p_temp, TRIO_CONFIG_WORD);
		if(p_data != NULL){
			p_temp = strchr(p_temp, '\r');
			if(p_temp != NULL){
				*p_temp = '\0';
				result = ProcessReceivedData(p_data, commandResponse, COMMAND_SOURCE_SMS);
			/*	if(commandResponse.b_needToReset)
					NVIC_SystemReset();*/
			}
		}
	}
	return result;
}
/********************************************************************/
bool Extract_SmsOrigAddr(char *buffer, char *smsOrigAddrBuf)
{
	char *startPtr, *endPtr;
	char smsInfo[64];

	/* copy first 64 bytes of SMS header for processing */
	memcpy(smsInfo, buffer, sizeof(smsInfo));

	startPtr = strstr(smsInfo, "\"+") + 1;
	endPtr = strchr(startPtr,'"');

	if(startPtr !=  NULL && endPtr != NULL){
		*endPtr = '\0';
		strcpy(smsOrigAddrBuf, startPtr);
		return TRUE;
	}
	return FALSE;
}
/********************************************************************/
void Get_SocketStats(char *buffer)
{
	char *p_temp;

#ifdef T0_TELIT_GL865
	p_temp = strstr(buffer, RespSI);
	if(p_temp != NULL){
		gsm_info.gsm_stats.u16_totalBytesSent      = Get_ValueByParamIndex(p_temp, 2);
		gsm_info.gsm_stats.u16_totalBytesRecv      = Get_ValueByParamIndex(p_temp, 3);
		gsm_info.gsm_stats.u16_totalBytesBuffered  = Get_ValueByParamIndex(p_temp, 4);
		gsm_info.gsm_stats.u16_totalBytesNonAcked1 = Get_ValueByParamIndex(p_temp, 5);
	}
#elif defined (T0_QUECTEL_M66) || defined (T0_SIMCOM_SIM800C)
	p_temp = strstr(buffer, RespQISACK);
	if(p_temp != NULL){
		gsm_info.gsm_stats.u16_totalBytesSent      = Get_ValueByParamIndex(p_temp, 1);
		gsm_info.gsm_stats.u16_totalBytesNonAcked1 = Get_ValueByParamIndex(p_temp, 3);
	}
#endif
		if(gsm_info.gsm_stats.u16_totalBytesNonAcked1 < gsm_info.gsm_stats.u16_totalBytesNonAcked2)
				i_tempNonAckedBytes = 0;
		else
				i_tempNonAckedBytes += gsm_info.gsm_stats.u16_totalBytesNonAcked1 - gsm_info.gsm_stats.u16_totalBytesNonAcked2;

		gsm_info.gsm_stats.u16_totalBytesNonAcked2 = gsm_info.gsm_stats.u16_totalBytesNonAcked1;

}
/********************************************************************/
void Get_SocketStatus(char *buffer)
{
	char *p_temp;

#ifdef T0_TELIT_GL865
	p_temp = strstr(buffer, RespSS);   /* get socket status*/
	if(p_temp != NULL){
		if(Get_ValueByParamIndex(p_temp, 2) == 0)
			gsm_info.b_socketActive = FALSE;
		else
			gsm_info.b_socketActive = TRUE;
	}
#elif defined (T0_QUECTEL_M66) || defined (T0_SIMCOM_SIM800C)
	p_temp = strstr(buffer, RespConnectOK);   /* get socket status*/
	if(p_temp != NULL)
		gsm_info.b_socketActive = TRUE;
	else
		gsm_info.b_socketActive = FALSE;
#endif
}
/*****************************************************************************/
#define MAX_TCP_READ_BLOCK_SIZE     256
#define MAX_SRECV_STRING_SIZE       44

uint16_t Get_TcpDataSegment(char *buffer, uint16_t recvLength, void (*callback)())
{
	uint16_t length;
	char cmdBuffer[64];
	char tempBuffer[300];
	char temp[8];
	char *pnewline;

	memset(tempBuffer, 0, sizeof(tempBuffer));
	memset(cmdBuffer, 0, sizeof(cmdBuffer));
	memset(buffer, 0, recvLength);

	itoa(recvLength, temp, 10);
	strcat((char *)cmdBuffer, Command_RecvData);
	strcat((char *)cmdBuffer, temp);
	strcat((char *)cmdBuffer,"\r");

	Send_ATCommand(cmdBuffer, "");
	length = ProcessATResponse(tempBuffer, sizeof(tempBuffer), 200, callback);

#ifdef T0_TELIT_GL865
	int i;
	char *ptrData;
	ptrData = strstr(tempBuffer, RespSRECV);
	if(ptrData != NULL){
		pnewline = strstr(ptrData, CHAR_ENTER);
			if(pnewline != NULL){
				length = Get_ValueByParamIndex(tempBuffer, 2);
					for(i = 0; i < length; i++)
						buffer[i] = *(pnewline + 2 + i);  /* move data to output buffer*/
					return length;
			}
			else return 0;
	}
	else
		return 0;
#elif defined(T0_QUECTEL_M66)
		char *ptemp1;

		/* find the length of the message between  ",TCP," and "\r\n"  */
		ptemp1 = strstr(tempBuffer, QUECTEL_TCP_HDR);
		pnewline = strstr(ptemp1, CHAR_ENTER);

		if((ptemp1 != NULL) && (pnewline != NULL)) {
			ptemp1 = ptemp1 + sizeof(QUECTEL_TCP_HDR);
		   *pnewline = '\0';
			length = atoi(ptemp1);
			memcpy(buffer, (pnewline + 2), length);

			PRINT_K(buffer);
			return length;
		}
		else
			return (0);
#endif
}

/*****************************************************************************/
uint16_t Get_GsmTcpBuffer(uint8_t *buffer, uint16_t length, void (*callBackFunc)())
{
	uint16_t dataoffset = 0;
	uint16_t u16_segmentSize;
	char segmentBuffer[256];

	memset(buffer, 0, length);

	if(length < MAX_TCP_RECEIVE_SIZE){
		u16_segmentSize = Get_TcpDataSegment(buffer, length, callBackFunc);
		return u16_segmentSize;
	}
	else{
#if defined(T0_TELIT_GL865) || defined(T0_SIMCOM_SIM800C)
		while(Get_NumberOfBufferedTcpBytes(callBackFunc) > 0){
			u16_segmentSize = Get_TcpDataSegment(segmentBuffer, MAX_TCP_RECEIVE_SIZE, callBackFunc);
			memcpy(&buffer[dataoffset], segmentBuffer, u16_segmentSize);
			dataoffset += u16_segmentSize;
	    }
#elif defined(T0_QUECTEL_M66)
	do{
		u16_segmentSize = Get_TcpDataSegment(segmentBuffer, MAX_TCP_RECEIVE_SIZE, callBackFunc);
		memcpy(&buffer[dataoffset], segmentBuffer, u16_segmentSize);
		dataoffset += u16_segmentSize;
	}while(u16_segmentSize > 0);
#endif
		return dataoffset;
	}
}
/*****************************************************************************/
uint16_t Get_NumberOfBufferedTcpBytes(void (*callBackFunc)())
{
	 char buffer[64];

	  uint16_t u16_response;
#if defined(T0_TELIT_GL865)
	  Send_ATCommand("AT#SI=1\r","");
#elif  defined(T0_SIMCOM_SIM800C)
	  Send_ATCommand("AT+CIPRXGET=4\r","");
#endif
	u16_response = ProcessATResponse(buffer, sizeof(buffer), 200, callBackFunc);
	if(u16_response > 0) {
#if defined(T0_TELIT_GL865)
		char *p_temp;
		p_temp = strstr((char *)buffer, RespSI);
		if(p_temp != NULL)
			return Get_ValueByParamIndex(p_temp, 4);
		else return 0;
#elif defined(T0_SIMCOM_SIM800C)
		p_temp = strstr((char *)buffer, "+CIPRXGET:");
		if(p_temp != NULL)
			return Get_ValueByParamIndex(p_temp, 2);
#endif
	}
	return 0;
}
/**************************************************************************/
void gsm_set_led_status(bool status)
{
	char temp[16];

	strcpy(temp, Command_SLED);

	if(status)
		strcat(temp,"1\r");
	else
		strcat(temp,"0\r");

	Send_ATCommand(temp,"");
	ProcessATResponse(temp, sizeof(temp), 50, onIdle);
}
/************************************************************************/
void gsm_led_blink()
{
	char temp[20];
    strcpy(temp, Command_SLED);
    strcat(temp, "3,30,70\r");
    Send_ATCommand(temp,"");
	ProcessATResponse(temp, sizeof(temp), 50, onIdle);
}
/**************************************************************************/
int16_t GetBatteryVoltage()
{
	return gsm_info.batteryLevel;
}
/**************************************************************************/
bool Get_GsmHealtStatus()
{
	return gsm_info.b_gsmHealtStatus;
}
/**************************************************************************/
#ifdef  T0_QUECTEL_M66
uint16_t Get_QuectelTCPPacket(char *buffer)
{
	uint16_t length;
	char *ptemp1, *ptemp2;

	/* find the length of the message between  ",TCP," and "\r\n"  */
	ptemp1 = strstr(buffer, QUECTEL_TCP_HDR);
	ptemp2 = strstr(buffer, CHAR_ENTER);

	if((ptemp1 != NULL) && (ptemp2 != NULL)) {
		ptemp1 = ptemp1 + sizeof(QUECTEL_TCP_HDR);
	   *ptemp2 = '\0';
		length = atoi(ptemp1);
		memcpy(buffer, (ptemp2 + 2), length);
		return length;
	}
	else
		return (0);
}
#endif
/****************************************************************************/
void onIdle()
{
	Trio_GpsTask();
	Trio_StatusTask();
	Trio_SPITask();
	Trio_DebugPortTask();
/*	Update_LedStatus();*/
}
