#ifndef SPI_H
#define SPI_H

#define CHAR_STX                 0x02
#define CHAR_ETX                 0x03
#define T_MESSAGE_START_CHAR     '['
#define T_MESSAGE_END_CHAR       ']'

typedef enum
{
	SPI_WRITE_SUCCESS,
	SPI_ERROR_ALLOCATING_MEMORY,
	SPI_ERROR_CRC
}SPI_RESULT_T;

typedef struct
{
	uint32_t lastWrittenAddr;
	uint32_t lastReadAddr;
}FLASH_T;
void Trio_SPITask();
void Trio_SPI_Init();
void SST25_WriteEnable();
void SST25_EnableWSR();
void SST25_WriteSR(uint8_t);
uint8_t SST25_ReadSR();
int Get_OffLineDataFromFlash(char *buffer);
bool SST25_Jedec_ID_Read();
void send_spi_data( LPC_SPI_T *pSPI,
					uint8_t *p_writeBuffer,
				    uint16_t u8_numBytesToBeWritten,
				    uint8_t *p_readBuffer,
				    uint16_t u8_numBytesToBeRead );

void CheckOffLineDataRecordTimeout();
void SST25_Write(uint32_t  address, uint16_t data);

SPI_RESULT_T SST25_Read(uint32_t address, uint16_t data, uint8_t *buffer);
SPI_RESULT_T MMA7455_Read(uint32_t address, uint16_t length, uint8_t *buffer);

uint32_t SST25_WriteArray(uint8_t *array, uint16_t length, uint32_t flashAddress);

void SST25_WriteByte(uint32_t , uint8_t);
bool SST25_ReadBUSY();
bool Get_FlashHealtStatus();
void SST25_EraseSector(uint32_t sector_no);
void SST25_ChipErase();
bool SST25_IsEmpty();
uint32_t GetFlashReadPointer();
uint32_t Get_FlashWriteAddress();
int32_t  ReadOffLineDataFromFlash(char *buffer);
uint32_t Get_FlashWriteAddress();
uint32_t Get_FlashReadAddress();
bool UpdateKmCounter(uint32_t u32_kmValue);
void AddOffLineRecord(uint8_t *, uint32_t);
void AddOfflineDataToCache(uint8_t *, uint16_t);
void WriteInitialValuesToFlash();
void ReloadLogTimer();
void Init_SPI_Cache();
void Load_UserSettings();
void Load_KmCounter();
void ReadStatusReg(char *msg);
#endif
