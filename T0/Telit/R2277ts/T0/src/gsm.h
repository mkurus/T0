
#ifndef GSM_UART_H
#define GSM_UART_H

#define GSM_POWER_ON_OFF_PIN     10
#define GSM_RESET_PIN            21

#ifdef  T0_TELIT_GL865
#define HARD_RESET_LOW_TIME      25
#define WAIT_TIME_AFTER_RESET    700
#elif   defined(T0_QUECTEL_M66) || defined(T0_SIMCOM_SIM800C)
#define HARD_RESET_LOW_TIME      (2 * SECOND)
#define WAIT_TIME_AFTER_RESET    70
#define QUECTEL_TCP_HDR   "TCP"
#endif


#define MAX_SMS_LENGTH             160
#define DEST_ADDR_LENGTH           24
#define M66_RESTART_DELAY_TIME     60

void Update_LedStatus();
void ExecuteMainLoop();
void Trio_Init_GSM_UART();
void Init_GSM_UART_PinMux();

#define SEND_FAIL_TIMEOUT                    (2 * HOUR)
#define SMS_SENT_RESPONSE_TIMEOUT            (10 * SECOND)

#define IMSI_LEN       15
#define IMEI_LEN       15
#define SMS_BUFFER_LEN 160

#define VERSION_T2

#ifndef VERSION_T2
#define VERSION_T0
#endif
#define  SECURITY_ENABLED_CHAR     ':'

typedef enum{
	TRACKING_SERVER =1,
	FIRMWARE_UPDATE_SERVER
}SERVER_TYPE_T;
/***********/
typedef struct SERVER_INFO{
	char server_ip[128];
	char server_port[6];
	SERVER_TYPE_T serverType_t;
}SERVER_INFO_T;

typedef union TCP_SERVER_INFO{
	SERVER_INFO_T firmwareUpdateServerInfo_t;
	SERVER_INFO_T trackingServerInfo_t;
}TCP_SERVER_INFO_T;

typedef struct TCP_CONNECTION_INFO{
	SERVER_INFO_T tcpServerInfo_t;
	uint8_t u8_connectionRetryCount;
}TCP_CONNECTION_INFO_T;

typedef struct GSM_STATS
{
	uint16_t u16_totalBytesSent;
	uint16_t u16_totalBytesRecv;
	uint16_t u16_totalBytesBuffered;
	uint16_t u16_totalBytesNonAcked1;
	uint16_t u16_totalBytesNonAcked2;
}GSM_STATS_T;

typedef struct GSM_INFO
{
	int csq;
	GSM_STATS_T gsm_stats;
	char imei_no[IMEI_LEN + 1];
	char imsi_no[IMSI_LEN + 1];
	bool b_gsmRegistered;
	bool b_roaming;
	bool b_socketActive;
	bool b_gsmHealtStatus;
	int16_t batteryLevel;
	uint8_t u8_commandRetryCount;
}GSM_INFO_T;

typedef enum GSM_DATA_TYPE{
	DATA_TYPE_NO_DATA,
	DATA_TYPE_IMEI_NUMBER,
	DATA_TYPE_IMSI_NUMBER,
	DATA_TYPE_SMS
}GSM_DATA_TYPE_T;


/*******************************************************/
typedef enum {
	MODULE_RESET_STATE,
	MODULE_INITIALIZING_STATE,
	MODULE_ATTACHING_TO_GPRS_STATE,
	MODULE_CONNECTING_TO_SERVER_STATE,
	MODULE_CONNECTED_TO_SERVER_STATE,
	MODULE_CONNECTED_TO_UPDATE_SERVER_STATE,
	MODULE_WAIT_IN_ROAMING_STATE
}GSM_MODULE_STATE;
TIMER_TICK_T Get_DataSendPeriod();
void TogglePin(uint8_t pin_number, uint16_t duration);
void GSM_PowerOff();
void GSM_PowerOn();
void onIdle(void);
bool Gsm_CloseSocket();
bool Send_MessageToServer(char *pBuffer, uint16_t i_msgLen, void (*onIdleCallback)());
/* function prototypes*/
void Init_GsmInfo();
bool Get_GsmHealtStatus();
void Get_GsmInfo(GSM_INFO_T *const );
int16_t GetBatteryVoltage();
void Set_GsmResetPinFunction(uint8_t pin_number);
void DeassertGsmResetPin(void);
bool Send_SMS(char *destAddr, char *smsText, void (*onIdleCallback)());
bool GetServerConnectionStatus();
void Set_ServerConnectionParameters(char *server_ip, char *server_port);
void Set_FirmwareUpdateServerConnection(char *ipAddress, char *tcpPort);
void Get_UpdateServerParameters(TCP_CONNECTION_INFO_T *serverInfo_t);
void Set_TrackingServerConnection(char *ipAddress, char *tcpPort);
uint16_t Get_TcpDataSegment(char * buffer, uint16_t length, void (*callback)());
uint16_t Get_GsmTcpBuffer(uint8_t *buffer, uint16_t length, void (*callBackFunc)());
uint16_t Get_NumberOfBufferedTcpBytes(void (*callBackFunc)());
bool ExtractIMEI(char *buffer, char *imei);
void gsm_set_led_status(bool status);
void gsm_led_blink();
#endif /*GSM_UART_H*/
