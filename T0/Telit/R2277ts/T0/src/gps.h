
#ifndef GPS_H
#define GPS_H

#define SIM28MLH

#if     defined(SIM28MLH)
#define GPS_BAUD_RATE             115200
#define GPS_MODULE_CODE           "s"
#elif   defined(QUECTEL_L70R)
#define GPS_BAUD_RATE   		  9600
#define GPS_MODULE_CODE           "q"
#endif

void Disable_GPS();
void Init_GPS_UART_Pinmux();
bool Trio_Init_GPS_UART();
void nmea_parse_next_field(const char **buffer, char seperator);
void ReloadPeriodicDataSendTimer();
/* Scalar for all fractional parsed values; actual fraction is the integer
 * fractional value divided by this constant.
 */
#define NMEA_FRACTIONAL_SCALER  100000
/** Multiplication factor of processed linear GPS latitude and longitude
 *  coordinates. Divide by this constant to obtain the integer number of
 *  degrees, modulus this value to get the fractional number of degrees.
*/
#define GPS_POS_DEGREES_SCALER  100000
/* 1 knot = 1.852 kmh */
#define METERS_IN_KNOTS   1852
#define MIN_NUMBER_OF_SATS_FOR_MOVING_NOT_IGNITED  5

#define CHAR_CR   '\r'
#define CHAR_LF   '\n'

#define IDLE_SPEED_THRESHOLD          2
#define GPS_BAUDRATE_DETECT_DELAY     300
typedef struct GPS_POSITION_DATA {
	/** Latitude  as a number of degrees scaled by  GPS_POS_DEGREES_SCALER */
	int32_t latitude;
	/** Longitude as a number of degrees scaled b  GPS_POS_DEGREES_SCALER */
	int32_t longitude;
	/** Altitude  as a number of absolute meters */
	uint16_t altitude;
	/* distance */
	double distance;
}GPS_POSITION_DATA_T;

/* Enum for mode 1 GPGSA */
enum nmea_gpgsa_mode1 {
	/** Manual, forced to operate in 2D or 3D mode */
	NMEA_GPGSA_MODE1_MANUAL    = 'M',
	/** 2D automatic, allowed to automatically switch 2D/3D */
	NMEA_GPGSA_MODE1_AUTOMATIC = 'A',
};

/* Enum for mode 2 GPGSA */
enum nmea_gpgsa_mode2 {
	/** Fix not available */
	NMEA_GPGSA_MODE2_NO_FIX = 1,
	/** 2D */
	NMEA_GPGSA_MODE2_2D     = 2,
	/** 3D */
	NMEA_GPGSA_MODE2_3D     = 3,
};

enum nmea_data_type {
	/** Unknown input */
	NMEA_TYPE_UNKNOWN,
	/** Recommended Minimum Specific GNSS Data */
	NMEA_TYPE_GPRMC,
	/** Time, position and fix type data */
	NMEA_TYPE_GPGGA,
	/** GPS receiver operating mode, active satellites used in the position
	 *  solution and DOP values */
	NMEA_TYPE_GPGSA,
};

typedef enum NMEA_GPRMC_STATUS{
	/* valid sentence */
	NMEA_GPRMC_VALID = 'A',
	/* invalid sentence */
	NMEA_GPGSA_INVALID = 'V'
}NMEA_GPRMC_STATUS_T;

/* Enum for the NMEA GPS indicators */
enum nmea_gps_coord_indicator {
	/** North */
	NMEA_GPS_COORD_NORTH = 'N',
	/** South */
	NMEA_GPS_COORD_SOUTH = 'S',
	/** West */
	NMEA_GPS_COORD_WEST  = 'W',
	/** East */
	NMEA_GPS_COORD_EAST  = 'E'
};

/* Structure for GPS latitude and longitude coordinates */
struct nmea_gps_coord_val {
	/** Indicator (N, S, E, W) */
	enum nmea_gps_coord_indicator indicator;
	/** Degrees */
	int16_t degrees;
	/** Minutes */
	int16_t minutes;
	/** Minutes (fractional) */
	int32_t minutes_frac;
};

/* Structure for GPS coordinates, latitude and longitude */
struct nmea_gps_coord {
	/** Latitude */
	struct nmea_gps_coord_val latitude;
	/** Longitude */
	struct nmea_gps_coord_val longitude;
};

typedef struct RMC_MESSAGE {
	/* UTC hour */
	uint8_t utc_hour;
	/* UTC minute */
	uint8_t utc_minute;
	/* UTC second */
	uint8_t utc_second;
	/* status */
	NMEA_GPRMC_STATUS_T status;
	/* GPS coordinates */
	struct nmea_gps_coord coords;
	/* Speed, knots (integer)  */
	uint16_t speed;
	/* Speed, knots (fractional) */
	uint16_t speed_frac;
	/* Course (integer) */
	uint16_t course;
	/* Course (fractional) */
	uint16_t course_frac;
	/* Date (day) */
	uint8_t day;
	/* Date (month) */
	uint8_t month;
	/* Date (year) */
	uint8_t year;
}RMC_MESSAGE_T;

typedef struct GGA_MESSAGE {
	/** UTC hour */
	uint8_t utc_hour;
	/** UTC minute */
	uint8_t utc_minute;
	/** UTC second */
	uint8_t utc_second;
	/** GPS coordinates */
	struct nmea_gps_coord coords;
	/** Position fix indicator */
	uint8_t position_fix;
	/** Number of satellites used */
	uint8_t satellites;
	/** Altitude */
	int16_t altitude;
}GGA_MESSAGE_T;

/* Structure for GPGSA data
 */
typedef struct GSA_MESSAGE {
	/** Mode 1 */
	enum nmea_gpgsa_mode1 mode1;
	/** Mode 2 - 2D or 3D */
	enum nmea_gpgsa_mode2 mode2;
	/** Satellites used on the channels */
	uint8_t satellite[12];
	/** Position dilution of precision (integer)   */
	uint8_t pdop_int;
	/** Position dilution of precision (fractional) */
	uint8_t pdop_frac;
	/** Horizontal dilution of precision (integer) */
	uint8_t hdop_int;
	/** Horizontal dilution of precision (fractional) */
	uint8_t hdop_frac;
	/** Vertical dilution of precision (integer) */
	uint8_t vdop_int;
	/** Vertical dilution of precision (fractional) */
	uint8_t vdop_frac;
}GSA_MESSAGE_T;

typedef struct NMEA_MSG {

	enum nmea_data_type type;
	struct NMEA_MSG_ITEMS_T{

		RMC_MESSAGE_T gprmc;
		GGA_MESSAGE_T gpgga;
		GSA_MESSAGE_T gpgsa;
	}nmea_msg_items_t;

} NMEA_MSG_T;

typedef struct NMEA_STRINGS {
	char* string;
	bool (*func)(const char *, NMEA_MSG_T *);

}NMEA_STRINGS_T;


typedef enum GPS_BAUD_RATES{
	GPS_BAUD_RATE_4800 = 4800,
	GPS_BAUD_RATE_9600 = 9600,
	GPS_BAUD_RATE_19200 = 19200,
	GPS_BAUD_RATE_115200 = 115200
}GPS_BAUD_RATES_T;

void Init_GPSInfo();
void Init_PositionInfo();
void Get_RMCInfo(RMC_MESSAGE_T *);
void Get_GSAInfo(GSA_MESSAGE_T *);
void Get_GGAInfo(GGA_MESSAGE_T *);
void Get_PositionInfo(GPS_POSITION_DATA_T *const);
void send_mtk_command(const char *command);
void Trio_GpsTask();
bool Get_GpsHealtStatus();
bool ifKmRecordLimitExceeded();
bool IsMoving();
#endif
