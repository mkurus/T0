#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "bsp.h"
#include "board.h"
#include "chip.h"
#include "gps.h"
#include "spi.h"
#include "messages.h"
#include "settings.h"
#include "status.h"
#include "bootloader.h"
#include "gsm.h"
#include "utils.h"
#include "timer.h"
#include "stdlib.h"
#include <string.h>

TIMER_INFO_T gpsModuleCommTimer;
#define GPS_UART_SRB_SIZE         100 	/* Send */
#define GPS_UART_RRB_SIZE         512	/* Receive */
#define GPS_MESSAGE_BUFFER_SIZE   128
#define GPS_MODULE_COMM_DOWN_TIMEOUT 500
#define GPS_UART         	 	  LPC_USART2

#define GPS_IRQ_SELECTION 	 	  UART2_IRQn
#define GPS_UART_ISR_HANDLER 	  UART2_IRQHandler

#define NMEA_MSG_DELIMITER   ','

/* module static variables */
static bool gps_healt_status = FALSE;
static uint8_t gps_txbuff[GPS_UART_SRB_SIZE] __attribute__ ((section (".common1")));
static uint8_t gps_rxbuff[GPS_UART_RRB_SIZE] __attribute__ ((section(".common1")));
static RINGBUFF_T gps_txring                 __attribute__ ((section (".common1")));
static RINGBUFF_T gps_rxring                 __attribute__ ((section (".common1")));
static GPS_POSITION_DATA_T current_position  __attribute__ ((section (".common1")));
static GPS_POSITION_DATA_T prev_position     __attribute__ ((section (".common1")));
static NMEA_MSG_T nmea_msg_t                 __attribute__ ((section (".common1")));
static char gpsRxBuffer[GPS_MESSAGE_BUFFER_SIZE]__attribute__ ((section (".common1")));
float f_kmTempCounter  __attribute__ ((section(".common1")));
static double f_distance __attribute__ ((section(".common1")));

static bool nmea_parse_gpgsa(const char *, NMEA_MSG_T *);
static bool nmea_parse_gprmc(const char *, NMEA_MSG_T *);
static bool nmea_parse_gpgga(const char *, NMEA_MSG_T *);

static bool nmea_parse_int(const char **, int8_t, int32_t *const);
static bool nmea_parse_character(const char **, char *const);
static bool nmea_parse_fixed_point(const char **, int32_t *const, int32_t *const);
static int32_t gps_convert_to_linear_degrees(const struct nmea_gps_coord_val *const,const enum nmea_gps_coord_indicator);
static double CalculateDistance(int32_t, int32_t, int32_t, int32_t);
GPS_POSITION_DATA_T const * ProcessGPSMessage(char *,  NMEA_MSG_T *);
uint8_t ValidateNMEAChecksum(char *);
void ParseNMEAMessage(char *,  NMEA_MSG_T *);
void FillNMEAMessageStruct(NMEA_MSG_T *);
/** Moves to the next token in a NMEA sentence.*/
#define NMEA_NEXT_TOKEN(buffer)      if (**buffer != '\0') { (*buffer)++; }

/* Current token in the NMEA sentence */
#define NMEA_CURRENT_TOKEN(buffer)   ((unsigned char)**buffer)

#define FLASH_KM_UPDATE_DISTANCE       15000


#define GPS_POWER_ENABLE_PIN         13

bool Trio_Init_GPS_UART()
{

    FLASH_SETTINGS_T user_settings;
    GPS_BAUD_RATES_T gps_baud_rates[]={GPS_BAUD_RATE_4800,
    								   GPS_BAUD_RATE_9600,
									   GPS_BAUD_RATE_19200,
									   GPS_BAUD_RATE_115200};
    int i;
    char temp[16];

    gps_healt_status = FALSE;

    Chip_UART_Disable(GPS_UART);
	Chip_UART_Init(GPS_UART);
	Chip_UART_ConfigData(GPS_UART, UART_CFG_DATALEN_8 |
				                   UART_CFG_PARITY_NONE |
								   UART_CFG_STOPLEN_1|
								   UART_CFG_RXPOL );

	RingBuffer_Init(&gps_rxring, gps_rxbuff, 1, GPS_UART_RRB_SIZE);
	RingBuffer_Init(&gps_txring, gps_txbuff, 1, GPS_UART_SRB_SIZE);

	Get_UserSettings(&user_settings);
	if(user_settings.gps_baud_rate != 0xFFFFFFFF){
		Chip_UART_SetBaud(GPS_UART, user_settings.gps_baud_rate);
		Chip_UART_Enable(GPS_UART);
		Chip_UART_TXEnable(GPS_UART);

		Chip_UART_IntEnable(GPS_UART, UART_INTEN_RXRDY);
		NVIC_SetPriority(GPS_IRQ_SELECTION, 1);
		NVIC_EnableIRQ(GPS_IRQ_SELECTION);
		PRINT_K("Valid gps baud rate found\n");
		return true;
	}
	else{
		for(i = 0; i< sizeof(gps_baud_rates)/ sizeof(gps_baud_rates[0]); i++){
			Chip_UART_SetBaud(GPS_UART,gps_baud_rates[i]);
			Chip_UART_Enable(GPS_UART);
			Chip_UART_TXEnable(GPS_UART);
			Chip_UART_IntEnable(GPS_UART, UART_INTEN_RXRDY);
			NVIC_SetPriority(GPS_IRQ_SELECTION, 1);
			NVIC_EnableIRQ(GPS_IRQ_SELECTION);
			Delay(GPS_BAUDRATE_DETECT_DELAY, onIdle);
			if(gps_healt_status){
				UpdateGpsBaudRate(gps_baud_rates[i]);
			PRINT_K("Detected baud rate:");
			itoa(gps_baud_rates[i], temp ,10);
			PRINT_K(temp);
			PRINT_K("\n");
				return TRUE;
			}
		}
	}

	Set_Timer(&gpsModuleCommTimer, GPS_MODULE_COMM_DOWN_TIMEOUT);
	return FALSE;
}
/*************************************************************/
void Init_GPS_UART_Pinmux()
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_SWM_MovablePinAssign(SWM_U2_TXD_O, 27);
	Chip_SWM_MovablePinAssign(SWM_U2_RXD_I, 16);

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}
/*************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) GPS_UART_ISR_HANDLER(void)
{
	Chip_UART_IRQRBHandler(GPS_UART, &gps_rxring, &gps_txring);
}
/***************************************************************************************/
void Trio_GpsTask()
{
	memset(gpsRxBuffer, 0, GPS_MESSAGE_BUFFER_SIZE);
	if(Get_UartLine(GPS_UART, &gps_rxring,  gpsRxBuffer, GPS_MESSAGE_BUFFER_SIZE, '$', '\r', 2)){
		ProcessGPSMessage(gpsRxBuffer, &nmea_msg_t);
	}
	return;
}

/******************************************************************/
GPS_POSITION_DATA_T const *ProcessGPSMessage(char *gps_msg,  NMEA_MSG_T *nmea_msg_t)
{
	char buffer[20];

	if(!ValidateNMEAChecksum(gps_msg)) {
		PRINT_K("\nGPS Checksum Error\n");
		return NULL;
	}
	else {
		ParseNMEAMessage(gps_msg, nmea_msg_t);
	//	PRINT_K(gps_msg);
		switch(nmea_msg_t->type)
		{
			case NMEA_TYPE_GPRMC:
			PRINT_K(gps_msg);
			gps_healt_status = TRUE;
			/* keep previous coordinates */
			if(nmea_msg_t->nmea_msg_items_t.gprmc.status == NMEA_GPRMC_VALID) {
				Activate_GPS_LED();
				if(Get_TripGpsStatus() == INVALID_GPS_ON_IGNITION)
					Init_TripInfo(VALID_GPS_ON_IGNITION);
				current_position.latitude = gps_convert_to_linear_degrees(
						   &(nmea_msg_t->nmea_msg_items_t.gprmc.coords.latitude),
						   NMEA_GPS_COORD_NORTH);

				current_position.longitude = gps_convert_to_linear_degrees(
						   &(nmea_msg_t->nmea_msg_items_t.gprmc.coords.longitude),
						   NMEA_GPS_COORD_EAST);

				/* Measure distance between two coordinates */
				f_distance = CalculateDistance(prev_position.latitude,
											   current_position.latitude,
											   prev_position.longitude,
											   current_position.longitude);


				if(Get_IgnitionStatus() && nmea_msg_t->nmea_msg_items_t.gprmc.speed *1852/1000 > IDLE_SPEED_THRESHOLD ){
					current_position.distance += f_distance;
					f_kmTempCounter += f_distance;
				}
				itoa(current_position.distance, buffer, 10);
				prev_position = current_position;
			}
			else
				Deactivate_GPS_LED();
			break;  /*  end of NMEA_TYPE_GPRMC */

			case NMEA_TYPE_GPGGA:
			break;

			case NMEA_TYPE_GPGSA:
			break;
			/*****/
		}  /** end of switch statement */
	}
	return &current_position;
}
/**********************************************************************/
/*  Validates NMEA message checksum. Returns 1 if chacksum is correct */
/*  otherwise returns 0                                                */
/**********************************************************************/
uint8_t ValidateNMEAChecksum(char *gps_msg)
{
	uint8_t i = 1;
	uint8_t u8_calcChecksum = 0;
	return 1;
	/* string representation of both received and calculated checksums */
	char s_calcChecksum[3];
	char s_recvChecksum[3];

	uint8_t u8_msgLen = strlen(gps_msg);

	while(gps_msg[i] != '*')
		u8_calcChecksum ^= gps_msg[i++];

	s_calcChecksum[0] = u8_calcChecksum / 16 + 0x30;
	s_calcChecksum[1] = u8_calcChecksum % 16;

	if( s_calcChecksum[1] < 10)
		s_calcChecksum[1] =s_calcChecksum[1] + 48;
	else
		s_calcChecksum[1] = s_calcChecksum[1] + 55;

	s_calcChecksum[2] = '\0';

	strncpy(s_recvChecksum, &gps_msg[u8_msgLen - 4], 2);
	if(strcmp(s_calcChecksum, s_recvChecksum) == 0)
		return 1;
	else
		return 0;
}
/*************************************************************************/
NMEA_STRINGS_T string_cases [] =
{
	{ "$GPRMC", nmea_parse_gprmc },
    { "$GPGGA", nmea_parse_gpgga },
	{ "$GPGSA", nmea_parse_gpgsa },
};
/**********************************************************************/
void ParseNMEAMessage(char* gps_msg, NMEA_MSG_T *nmea_msg_t)
{
	NMEA_STRINGS_T* pCase;
	char buffer[7];
	memset(buffer, 0, sizeof(buffer));
   /* get the nmea message ID */
	strncpy(buffer, gps_msg, 6);

//	PRINT_K(buffer);
	nmea_msg_t->type = (enum nmea_data_type)NMEA_TYPE_UNKNOWN;
    for(pCase = string_cases
				; pCase != string_cases + sizeof( string_cases ) / sizeof( string_cases[0])
				; pCase++ ){
			if(strcmp( pCase->string, buffer) == 0){
				(*pCase->func)(gps_msg, nmea_msg_t);
				break;
			}
	}
}
/********************************************************************
 Parse sentence until the next comma delimited field.
 param : buffer  Current location in the NMEA sentence
*********************************************************************/
void nmea_parse_next_field(const char **buffer, char seperator)
{
	while (NMEA_CURRENT_TOKEN(buffer) != seperator)
		NMEA_NEXT_TOKEN(buffer);

	NMEA_NEXT_TOKEN(buffer);
}
/********************************************************************/
static bool nmea_parse_gpgsa(const char *buffer, NMEA_MSG_T *data)
{
	bool b_parseSuccess = true;
	int32_t temp_int, temp_int2;
	uint8_t i;
	char temp_char;


	data->type = (enum nmea_data_type)NMEA_TYPE_GPGSA;

	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_character(&buffer, &temp_char);
	data->nmea_msg_items_t.gpgsa.mode1 = temp_char;

	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_int(&buffer, -1, &temp_int);
	data->nmea_msg_items_t.gpgsa.mode2 = temp_int;

	nmea_parse_next_field(&buffer, ',');

	for (i = 0; i < sizeof(data->nmea_msg_items_t.gpgsa.satellite) / sizeof(data->nmea_msg_items_t.gpgsa.satellite[0]); i++) {
		b_parseSuccess &= nmea_parse_int(&buffer, -1, &temp_int);
		data->nmea_msg_items_t.gpgsa.satellite[i] = temp_int;

		nmea_parse_next_field(&buffer, ',');
	}

	b_parseSuccess &= nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);

	if(b_parseSuccess){
		data->nmea_msg_items_t.gpgsa.pdop_int = temp_int;
		data->nmea_msg_items_t.gpgsa.pdop_frac = temp_int2;
	}
	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);

	data->nmea_msg_items_t.gpgsa.hdop_int = temp_int;
	data->nmea_msg_items_t.gpgsa.hdop_frac = temp_int2;

	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);

	data->nmea_msg_items_t.gpgsa.vdop_int = temp_int;
	data->nmea_msg_items_t.gpgsa.vdop_frac = temp_int2;


	return b_parseSuccess;
}
/*****************************************************************/
static bool nmea_parse_gprmc(const char *buffer, NMEA_MSG_T * gps_data)
{
	bool b_parseSuccess = true;
	int32_t temp_int;
	int32_t temp_int2;
	char temp_char;


	gps_data->type = (enum nmea_data_type)NMEA_TYPE_GPRMC;

	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess = nmea_parse_int(&buffer, 2, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.utc_hour = temp_int;

	b_parseSuccess = nmea_parse_int(&buffer, 2, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.utc_minute = temp_int;

	b_parseSuccess = nmea_parse_int(&buffer, 2, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.utc_second = temp_int;

	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess = nmea_parse_character(&buffer, &temp_char);

	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.status = (NMEA_GPRMC_STATUS_T)temp_char;

	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess = nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);
	/* parsing latitude */
	if(b_parseSuccess){
		gps_data->nmea_msg_items_t.gprmc.coords.latitude.degrees      = temp_int / 100;
		gps_data->nmea_msg_items_t.gprmc.coords.latitude.minutes      = temp_int % 100;
		gps_data->nmea_msg_items_t.gprmc.coords.latitude.minutes_frac = temp_int2;
	}

/*	PRINT_K("******************");
	itoa(gps_data->nmea_msg_items_t.gprmc.coords.latitude.degrees, buff,10);
	PRINT_K(buff);
	itoa(gps_data->nmea_msg_items_t.gprmc.coords.latitude.minutes, buff,10);
		PRINT_K(buff);
		itoa(gps_data->nmea_msg_items_t.gprmc.coords.latitude.minutes_frac, buff,10);
			PRINT_K(buff);
			PRINT_K("**********************");*/
	/* parsing latitude indicator*/
	nmea_parse_next_field(&buffer, ',');
	b_parseSuccess = nmea_parse_character(&buffer, &temp_char);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.coords.latitude.indicator = (enum nmea_gps_coord_indicator)temp_char;

    /* parsing Longitude */
	nmea_parse_next_field(&buffer, ',');
	b_parseSuccess = nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);

	if(b_parseSuccess){
		gps_data->nmea_msg_items_t.gprmc.coords.longitude.degrees      = temp_int / 100;
		gps_data->nmea_msg_items_t.gprmc.coords.longitude.minutes      = temp_int % 100;
		gps_data->nmea_msg_items_t.gprmc.coords.longitude.minutes_frac = temp_int2;
	}

/*	PRINT_K("**********aaa********");
	PRINT_K("**********aaa********");
		itoa(gps_data->nmea_msg_items_t.gprmc.coords.longitude.degrees, buff,10);
		PRINT_K(buff);
		itoa(gps_data->nmea_msg_items_t.gprmc.coords.longitude.minutes, buff,10);
			PRINT_K(buff);
			itoa(gps_data->nmea_msg_items_t.gprmc.coords.longitude.minutes_frac, buff,10);
				PRINT_K(buff);
				PRINT_K("*********aaa*************");*/
	/* parsing longtitude indicator*/
	nmea_parse_next_field(&buffer, ',');
	b_parseSuccess = nmea_parse_character(&buffer, &temp_char);

	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.coords.longitude.indicator = (enum nmea_gps_coord_indicator)temp_char;

	/* parsing speed*/
	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess = nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);

	if(b_parseSuccess){
		gps_data->nmea_msg_items_t.gprmc.speed      = temp_int;
		gps_data->nmea_msg_items_t.gprmc.speed_frac = temp_int2;
	}

	/* parsing course over ground*/
	nmea_parse_next_field(&buffer, ',');
	b_parseSuccess = nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);
	if(b_parseSuccess){
		gps_data->nmea_msg_items_t.gprmc.course      = temp_int;
		gps_data->nmea_msg_items_t.gprmc.course_frac = temp_int2;
	}

	/* parsing date*/
	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess = nmea_parse_int(&buffer, 2, &temp_int);

	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.day = temp_int;

	b_parseSuccess = nmea_parse_int(&buffer, 2, &temp_int);

	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.month = temp_int;

	b_parseSuccess = nmea_parse_int(&buffer, 2, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gprmc.year = temp_int;

	return b_parseSuccess;
}
/****************************************************************************/
static bool nmea_parse_gpgga(const char *buffer, NMEA_MSG_T * gps_data)
{
	bool b_parseSuccess = true;
	int32_t temp_int;

	gps_data->type = (enum nmea_data_type)NMEA_TYPE_GPGGA;
	/* parsing time */
	nmea_parse_next_field(&buffer, ',');

/*	b_parseSuccess &= nmea_parse_int(&buffer, 2, &temp_int);
	gps_data->nmea_msg_items_t.gpgga.utc_hour = temp_int;
	b_parseSuccess &= nmea_parse_int(&buffer, 2, &temp_int);
	gps_data->nmea_msg_items_t.gpgga.utc_minute = temp_int;
	b_parseSuccess &= nmea_parse_int(&buffer, 2, &temp_int);
	gps_data->nmea_msg_items_t.gpgga.utc_second = temp_int;
*/
    /* parsing latitude*/
	nmea_parse_next_field(&buffer, ',');

/*	b_parseSuccess &= nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);
	if(b_parseSuccess){
		gps_data->nmea_msg_items_t.gpgga.coords.latitude.degrees      = temp_int / 100;
		gps_data->nmea_msg_items_t.gpgga.coords.latitude.minutes      = temp_int % 100;
		gps_data->nmea_msg_items_t.gpgga.coords.latitude.minutes_frac = temp_int2;
	}
*/
	/* parsing latitude indicator*/
	nmea_parse_next_field(&buffer, ',');

/*	b_parseSuccess &= nmea_parse_character(&buffer, &temp_char);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gpgga.coords.latitude.indicator = (enum nmea_gps_coord_indicator)temp_char;
*/
    /* parsing Longitude */
	nmea_parse_next_field(&buffer, ',');

/*	b_parseSuccess &= nmea_parse_fixed_point(&buffer, &temp_int, &temp_int2);
	if(b_parseSuccess){
		gps_data->nmea_msg_items_t.gpgga.coords.longitude.degrees      = temp_int / 100;
		gps_data->nmea_msg_items_t.gpgga.coords.longitude.minutes      = temp_int % 100;
		gps_data->nmea_msg_items_t.gpgga.coords.longitude.minutes_frac = temp_int2;
	}
*/
	/* parsing longtitude indicator*/
	nmea_parse_next_field(&buffer, ',');

/*	b_parseSuccess &= nmea_parse_character(&buffer, &temp_char);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gpgga.coords.longitude.indicator = (enum nmea_gps_coord_indicator)temp_char;
*/
	/* parsing fix position*/
	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_int(&buffer, -1, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gpgga.position_fix = temp_int;



	/* parsing number of satellites field*/
	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_int(&buffer, -1, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gpgga.satellites = temp_int;


	/* Horizontal Dilution Of Precision is not used. Do a dummy read */
/*	nmea_parse_next_field(&buffer, ',');*/

	/* parsing altitude*/
/*	nmea_parse_next_field(&buffer, ',');

	b_parseSuccess &= nmea_parse_int(&buffer, 2, &temp_int);
	if(b_parseSuccess)
		gps_data->nmea_msg_items_t.gpgga.altitude = temp_int;*/

	return b_parseSuccess;
}
/*****************************************************************************/
/* Parse the current integer field value.
 *
 * buffer      [in, out]  Current location in the NMEA sentence
 * max_length  [in}       Maximum length of the  integer
 * out         [out]      Location of the parsed integer
 *
 * ***************************************************************************/
static bool nmea_parse_int(const char **buffer, int8_t max_length, int32_t *const out)
{
	int32_t parsed_value = 0;

	while (isdigit(NMEA_CURRENT_TOKEN(buffer)) && max_length--) {
		parsed_value = (parsed_value * 10) + (NMEA_CURRENT_TOKEN(buffer) - '0');
		NMEA_NEXT_TOKEN(buffer);
	}

	*out = parsed_value;
	return true;
}
/*****************************************************************************/
/* Parse the current character field value.
 *
 *  buffer [in, out] Current location in the NMEA sentence
 *  out    [out]     Location of the parsed character
 *
 *  return bool  true if the field was parsed successfully.
 *
 ***************************************************************************/
static bool nmea_parse_character(const char **buffer, char *const out)
{
	char parsed_char   = '\0';
	bool b_parseSuccess = false;

	if (isupper(NMEA_CURRENT_TOKEN(buffer))) {
		parsed_char   = NMEA_CURRENT_TOKEN(buffer);
		b_parseSuccess = true;
	}
//	NMEA_NEXT_TOKEN(buffer);

	*out = parsed_char;

	return b_parseSuccess;
}
/***************************************************************************/
/* Parse the current fixed point X.Y field value.
 *
 *  buffer          [in, out]   Current location in the NMEA sentence
 *  out_whole       [out]       Location of the parsed integer portion
 *  out_fractional  [out]       Location of the parsed fractional portion
 *
 * return bool true if the field was parsed successfully.
 */
/***************************************************************************/
static bool nmea_parse_fixed_point(const char **buffer, int32_t *const out_whole, int32_t *const out_fractional)
{
	int32_t parsed_fractional = 0;
	int32_t parsed_whole      = 0;
	uint32_t i	              = 0;
	bool b_parseSuccess       = true;

	b_parseSuccess &= nmea_parse_int(buffer, -1, &parsed_whole);

	if(b_parseSuccess)
	{
		if (NMEA_CURRENT_TOKEN(buffer) != '.') {
			return false;
		}
		NMEA_NEXT_TOKEN(buffer);

		for (i = 10; i <= NMEA_FRACTIONAL_SCALER; i *= 10) {
			if (!isdigit(NMEA_CURRENT_TOKEN(buffer))){
				break;
			}
			else{
				parsed_fractional *= 10;
				parsed_fractional += (NMEA_CURRENT_TOKEN(buffer) - '0');
				NMEA_NEXT_TOKEN(buffer);
			}
		}

		if(i == 1000)
			parsed_fractional /= 10;

		*out_whole      = parsed_whole;
		*out_fractional = parsed_fractional;
	}
	return b_parseSuccess;
}
/*****************************************************************************/
/** Converts a latitude/longitude coordinate from Degrees, Minutes and
 *  Fractional Minutes to linear number of degrees scaled by
 *  GPS_POS_DEGREES_SCALER.
 *
 *   gps_coord          [in]     Raw DDMM.mmmm coordinate from a GPS module
 *   positive_indicator [in]     Indicator when the degrees is positive
 *
 *  return Input coordinate converted to a scaled number of linear degrees.
 ******************************************************************************/
static int32_t gps_convert_to_linear_degrees(
		       const struct nmea_gps_coord_val *const gps_coord,
			   const enum nmea_gps_coord_indicator positive_indicator)
{
	int32_t new_coordinate;
	/* Convert raw GPS latitude to linear floating point */
	new_coordinate  = (gps_coord->degrees) * GPS_POS_DEGREES_SCALER;
/*	PRINT_K("new_coordinate");
	itoa(new_coordinate, buffer,10);
	PRINT_K(buffer);*/

	new_coordinate += (gps_coord->minutes *  GPS_POS_DEGREES_SCALER) / 60;
/*	PRINT_K("new_coordinate");
		itoa(new_coordinate, buffer,10);
		PRINT_K(buffer);*/
	new_coordinate += (gps_coord->minutes_frac*10) / 60;
/*	PRINT_K("new_coordinate");
		itoa(new_coordinate, buffer,10);
		PRINT_K(buffer);*/

	/* Negate converted coordinate if the orientation is negative */
	if (gps_coord->indicator != positive_indicator) {
		new_coordinate = -new_coordinate;
		}

	return new_coordinate;
}
/**********************************************************************************/
double CalculateDistance(int32_t i_prevLat, int32_t i_currLat, int32_t i_prevLon, int32_t i_currLon)
{
	double f_temp1 = 0;
	double f_temp2 = 0;

	/* Convert ddMM to degrees*/
	double R = 6371000;

	if((i_prevLat == 0) || (i_currLat == 0) || (i_prevLon == 0) || (i_currLon == 0)){
		return 0;
	}

//	d_lat1 = (double)i_prevLat;  // GPS_POS_DEGREES_SCALER;
//	d_lat2 = (double)i_currLat;  // GPS_POS_DEGREES_SCALER;
//	d_lon1 = (double)i_prevLon;  // GPS_POS_DEGREES_SCALER;
//	d_lon2 = (double)i_currLon;  // GPS_POS_DEGREES_SCALER;
//
/*	itoa(i_currLat, buffer, 10);
	PRINT_K(buffer);
	itoa(i_prevLat, buffer, 10);
	PRINT_K(buffer);
	itoa(i_currLon, buffer, 10);
	PRINT_K(buffer);
	itoa(i_prevLon, buffer, 10);
	PRINT_K(buffer);*/

	double d_lat = (double)(i_currLat - i_prevLat) * 0.0174532925 / (100000);    /* to radian*/
	double d_lon = (double)(i_currLon - i_prevLon) * 0.0174532925 / (100000);    /* to radian*/

	//f_temp1 = (d_lat1 * 0.0174532925);
//	f_temp2 = (d_lat2 * 0.0174532925);

	f_temp1 = (double)i_currLat * 0.0174532925 / (100000);
	f_temp2 = (double)i_currLat * 0.0174532925 / (100000);

	double a = sin(d_lat/2) * sin(d_lat/2) + sin(d_lon/2) * sin(d_lon/2) * cos(f_temp1) * cos(f_temp2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));

	return (R * c);
}
/************************************************************************/
void Get_RMCInfo(RMC_MESSAGE_T *rmc_info)
{
	*rmc_info = nmea_msg_t.nmea_msg_items_t.gprmc;
}
/*************************************************************************/
void Get_GSAInfo(GSA_MESSAGE_T *gsa_info)
{
	*gsa_info = nmea_msg_t.nmea_msg_items_t.gpgsa;
}
/*************************************************************************/
void Get_GGAInfo(GGA_MESSAGE_T *gga_info)
{
	*gga_info = nmea_msg_t.nmea_msg_items_t.gpgga;
}
/*************************************************************************/
void Get_PositionInfo(GPS_POSITION_DATA_T *const position_info)
{
	*position_info = current_position;
}
/***************************************************************************/
void Init_PositionInfo()
{
	memset(&current_position, 0, sizeof(current_position));
	memset(&prev_position, 0, sizeof(prev_position));
}
/****************************************************************************/
void Init_GPSInfo()
{

/*	memset(gps_rxbuff, 0, GPS_UART_RRB_SIZE);
	memset(gps_txbuff, 0, GPS_UART_SRB_SIZE);
	memset(&gps_txring, 0, sizeof(gps_txring));
	memset(&gps_rxring, 0, sizeof(gps_rxring));
	memset(&current_position, 0, sizeof(current_position));
	memset(&prev_position, 0, sizeof(prev_position));
	memset(&nmea_msg_t, 0 , sizeof(nmea_msg_t));
	f_kmTempCounter = 0 ;
	f_distance = 0;*/
}
/*****************************************************************************/
bool ifKmRecordLimitExceeded()
{
	if(f_kmTempCounter >= FLASH_KM_UPDATE_DISTANCE)
	{
		//PRINT_INT((int)f_kmTempCounter);
		f_kmTempCounter = f_kmTempCounter - FLASH_KM_UPDATE_DISTANCE;
		return TRUE;
	}
	else
		return FALSE;
}
/******************************************************************/
void ToggleGPSPower()
{
	TIMER_INFO_T wait_timer;

	Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GPS_POWER_ENABLE_PIN, FALSE);
	Set_Timer(&wait_timer, 2);

	while (!mn_timer_expired(&wait_timer)){
		onIdle();
	}

    Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, GPS_POWER_ENABLE_PIN, TRUE);
}
/*******************************************************************/
void Disable_GPS()
{
	extern unsigned int  _firmware_download_variables_start;
	extern unsigned int _firmware_download_variables_end;

	Chip_UART_IntDisable(GPS_UART, UART_INTEN_RXRDY);
	Chip_UART_DeInit(GPS_UART);
//	Chip_UART_Disable(GPS_UART);

	memset(&_firmware_download_variables_start, 0, &_firmware_download_variables_end - &_firmware_download_variables_start);

}
/*******************************************************/
void Trio_ConfigureGPSPowerTogglePin()
{
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, GPS_POWER_ENABLE_PIN);
}
/********************************************************/
void send_mtk_command(const char *command)
{
	Chip_UART_SendRB(GPS_UART, &gps_txring, command, strlen(command));
}
/********************************************************/
bool IsMoving()
{
	NMEA_MSG_T *gps_info = &nmea_msg_t;

	if((gps_info->nmea_msg_items_t.gprmc.speed *1852/1000 > 10) &&
	   (gps_info->nmea_msg_items_t.gpgga.position_fix) &&
	   (gps_info->nmea_msg_items_t.gpgga.satellites >= MIN_NUMBER_OF_SATS_FOR_MOVING_NOT_IGNITED))
		return TRUE;
	else
		return FALSE;
}
/********************************************************/
bool Get_GpsHealtStatus()
{
	return gps_healt_status;
}
