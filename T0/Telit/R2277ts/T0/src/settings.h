
#ifndef SETTINGS_H_
#define SETTINGS_H_

/* macro definitions for parameters residing in flash */
#define SIZEOF(s,m)          ((size_t)sizeof(((s *)0)->m))
#define READ_PARAMETER(M,D)  read_flash_parameter(offsetof(FLASH_SETTINGS_T,M),SIZEOF(FLASH_SETTINGS_T,M), D)
#define WRITE_PARAMETER(M,D) write_flash_parameter(offsetof(FLASH_SETTINGS_T,M),SIZEOF(FLASH_SETTINGS_T,M), D)

#define FLASH_SETTINGS_TABLE_BASE_ADDR    0x00000000

#define MAX_APN_NAME_LEN           24
#define MAX_APN_USERNAME_LEN       12
#define MAX_APN_PASSWORD_LEN       12
#define MAX_SERVER_ADDR_LEN        128
#define MAX_SET_COMMAND_PSWD_LENGTH          12


#define ADMIN_PASSWORD           "17RPT"


/* do not add padding bytes to struct members */
typedef struct //__attribute__ ((__packed__))
{
	char flash_apn[MAX_APN_NAME_LEN];
	char flash_apnusername[MAX_APN_USERNAME_LEN];
	char flash_apnpassword[MAX_APN_PASSWORD_LEN];
	char server_ip[MAX_SERVER_ADDR_LEN];
	char server_port[6];
	char device_id[16];
	char password[MAX_SET_COMMAND_PSWD_LENGTH+ 1];
	bool blockage_status_in_ram;
	bool roaming_activation;
	bool sms_activation;
	bool engine_blockage;
	uint32_t msg_period_roaming_ignited;
	uint32_t msg_period_roaming_not_ignited;
	uint32_t msg_period_not_roaming_ignited;
	uint32_t msg_period_not_roaming_not_ignited;
	uint16_t u16_speedLimit;
	uint16_t u16_speedLimitViolationDuration;
	uint32_t u32_maxStopTime;
	uint32_t u32_maxIdleTime;
	uint32_t gps_baud_rate;
	uint32_t chksum;
}FLASH_SETTINGS_T;

typedef enum {
	SERVER_AND_PORT_SETTING = 1,
	RESERVED1,
	RESERVED2,
	MESSAGE_PERIOD_SETTING,
	ROAMING_ACTIVATION,
	SMS_ACTIVATION,
	SPEED_LIMIT_SETTING,
	ENGINE_BLOCKAGE_SETTING,
	RESERVED3,
	RESERVED4,
	KM_COUNTER,
	DEVICE_RESET,
	RESERVED5,
	SEND_SMS,
	DEVICE_ID_SETTING,
	RESERVED6,
	RESERVED7,
	RESERVED8,
	DEVICE_STATUS_REQUEST,
	DIGITAL_INPUT_ALARM_SETTING,
	RESERVED9,
	UNCONTROLLED_ENGINE_BLOCKAGE,
	LOCATION_REQUEST,
	RESERVED10,
	APN_SETTING,
	ENGINE_OPERATION_SETTINGS,
	RESERVED11,
	GARMIN_FMI_ACTIVATION,
	TEXT_MESSAGE_TO_GARMIN,
	RESERVED12,
	RESERVED13,
	CHANGE_PASSWORD,
	GPS_BAUDRATE_SETTING=40,
	FIRMWARE_UPDATE_REQUEST = 503,
	IDLE_ALARM_SETTING = 600,
	GPS_COLD_RESTART = 1000,
	ERASE_EXT_FLASH = 1001,
	SET_BLOCKAGE_PERSISTANCE = 1002
}COMMAND_TYPE_T;

typedef enum {
	DEVICE_HEALT_STATUS
}GET_COMMAND_TYPE;
typedef enum{
	REPLY_DO_NOT_SEND,
	REPLY_ECHO_PACKET,
	REPLY_CONNECT_TO_UPDATE_SERVER
}COMMAND_RESULT_T;

typedef enum {
	COMMAND_SOURCE_SMS,
	COMMAND_SOURCE_TCP,
	COMMAND_SOURCE_UART
}COMMAND_SOURCE_T;

typedef struct SETTING_INFO
{
	COMMAND_TYPE_T command;
	COMMAND_RESULT_T (*update_callback)(char *const buffer, COMMAND_RESPONSE_T *response);
}SETTING_INFO_T;

typedef struct GET_CMD_INFO
{
	GET_COMMAND_TYPE command;
	COMMAND_RESULT_T (*get_callback)(char *const buffer, COMMAND_RESPONSE_T *response);
}GET_CMD_INFO_T;

typedef enum{
	BLOCKAGE_NOT_EXIST,
	BLOCKAGE_REQUESTED,
	BLOCKAGE_REMOVE_REQUESTED,
	BLOCKAGE_ACTIVATED,
	BLOCKAGE_REMOVED
}BLOCKAGE_STATUS_T;

typedef struct BLOCKAGE_INFO
{
	COMMAND_SOURCE_T cmdSource;
	BLOCKAGE_STATUS_T blockageStatus;
	char transactionID[16];
}BLOCKAGE_INFO_T;
void print_settings();
void write_default_settings();
bool check_settings_integrity(FLASH_SETTINGS_T *settings);
BLOCKAGE_STATUS_T Get_BlockageStatus();
void Set_BlockageStatus(BLOCKAGE_STATUS_T new_status);
bool isRoamingActivated();
uint16_t Get_MaxSpeedLimit();
uint16_t Get_MaxSpeedViolationDuration();
void Load_BlockageSetting();
uint32_t Get_MaxStopTime();
void Load_UserSettings();
void Update_Settings();
void UpdateGpsBaudRate(uint32_t gps_baud_rate);
void Get_UserSettings(FLASH_SETTINGS_T *user_settings);
uint32_t Get_FlashKmValue();
COMMAND_RESULT_T ParseConfigurationString(char *const buffer, COMMAND_RESPONSE_T *response, COMMAND_SOURCE_T cmdSource);
COMMAND_RESULT_T ParseGetCommand(char *const buffer, COMMAND_RESPONSE_T *response, COMMAND_SOURCE_T cmdSource);
COMMAND_RESULT_T ProcessReceivedData(char *recv_data, COMMAND_RESPONSE_T *response, COMMAND_SOURCE_T cmdSource);
bool IsPasswordCorrect(char *pswd);
void BeginEchoPacket(char *buffer, COMMAND_TYPE_T command);
void BeginGetResponse(char *pBuf, COMMAND_TYPE_T command);
void CloseEchoPacket(char *buffer);
void AddStringToEchoPacket(char *buffer, char *strToAdd);
int32_t PrepareBlockageEchoPacket(char *buffer);
#endif /* SETTINGS_H_ */
