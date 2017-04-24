#include "board.h"
#include "timer.h"
#include "messages.h"
#include "settings.h"
#include "status.h"
#include "utils.h"
#include "spi.h"
#include "sst25.h"
#include "gsm.h"
#include "gps.h"

#include "bootloader.h"
#include <stdlib.h>
#include <string.h>
static void setupMaster(void);

#define MAX_LOG_ENTRY_SIZE   256

static void Init_SPI_PinMux(void);
void  UpdateOffLineDataWriteAddress();
void  UpdateOffLineDataReadAddress();
uint16_t Get_CurrentWriteSectorNumber();
uint16_t Get_CurrentReadSectorNumber();
uint16_t Get_WriteEndSectorNumber();
uint16_t Get_NextWriteSectorNumber();
uint16_t Get_EndWriteSectorNumber();
uint16_t Get_ReadBlockSize();
uint32_t Get_OffLineDataWriteAddress();
uint32_t Get_OffLineDataReadAddress();
TIMER_TICK_T  Get_LogPeriod();
void SPI0_IRQHandler(void);
void SPI1_IRQHandler(void);
void FlushOffLineDataBuffer();
void Init_SPI_Cache();
bool check_settings_integrity(FLASH_SETTINGS_T *settings);
static void wait_spi_busy_timeout(TIMER_TICK_T timeout);
volatile bool b_transferDone;

TIMER_INFO_T OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMER;
TIMER_INFO_T OFFLINE_DATA_RECORD_TIMER;
TIMER_INFO_T OFFLINE_DATA_FLUSH_TIMER;

#define MSG_HEADER_SIZE           4
#define SPI_PROCESS_TIMEOUT       50

#define OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMEOUT     (1  * HOUR)
#define OFFLINE_DATA_FLUSH_TIMEOUT                   (30 * MINUTE)

/* Buffer sizes for SPI transfers */

#define  SPI_FLASH_RX_BUFFER_SIZE   512
#define  SPI_FLASH_TX_BUFFER_SIZE   16
#define  FLASH_BUSY  0x01



#define  SST25_FLASH_SECTOR_MASK			     0x00000FFF
#define  SST25_FLASH_SIZE                        (1024 * 1024)
#define  SST25_SECTOR_COUNT                      (SST25_FLASH_SIZE / SST25_SECTOR_SIZE)
#define  ADDR_OF_NEXT_FLASH_LOCATION             (SST25_FLASH_SIZE - FLASH_SECTOR_SIZE)
#define  ADDR_OF_NEXT_FLASH_LOCATION_MIRRORED    (SST25_FLASH_SIZE - 2 * FLASH_SECTOR_SIZE)

#define  OFFLINE_IGNITED_LOG_PERIOD               (15 * SECOND)
#define  OFFLINE_NOT_IGNITED_LOG_PERIOD           (15 * MINUTE)


#define  OFFLINE_DATA_CACHE_SIZE           	 1024

typedef struct
{
	uint32_t u32_offLineDataWriteAddress;
	uint32_t u32_offLineDataReadAddress;
	uint32_t u32_offLineDataReadAddressPersisted;
	uint16_t bufferedBytes;
	uint8_t  buffer[OFFLINE_DATA_CACHE_SIZE];
}OFFLINE_DATA_T;

typedef struct LOG_ENTRY_INFO{
	char logBuffer[MAX_LOG_ENTRY_SIZE];
	uint16_t u16_forward;
	bool b_validEntry;
}LOG_ENTRY_INFO_T;

static int Get_LogEntry(LOG_ENTRY_INFO_T *log_info);
/* Master transmit and receive buffers */
//char flashTransferRXBuffer[OFFLINE_DATA_CACHE_SIZE + 4];
uint8_t flashTransferTXBuffer[SPI_FLASH_TX_BUFFER_SIZE];

/* SPI master transfer descriptor */
SPIM_XFER_T spiFlashXfer;
SPIM_XFER_T spiAccelXfer;
SPIM_XFER_T *spiXfer;

#define SPIFLASHIRQHANDLER                 SPI0_IRQHandler
#define SPIACCELIRQHANDLER                 SPI1_IRQHandler

#define LPC_SPIFLASHPORT                   LPC_SPI0
#define LPC_SPIACCELPORT                   LPC_SPI1

#define LPC_SPIFLASHIRQNUM                 SPI0_IRQn
#define LPC_SPIACCELIRQNUM                 SPI1_IRQn

#define SPIFLASHCLOCKRATE                  (100000)
#define SPIACCELCLOCKRATE                  (100000)

static OFFLINE_DATA_T offLineData  __attribute__ ((section (".common1")));
static bool b_flashHealtStatus = FALSE;
void Trio_SPI_Init()
{
	 /* SPI initialization */
	/* memset(&spiFlashXfer, 0,  sizeof(spiFlashXfer));
	 memset(flashTransferTXBuffer, 0, SPI_FLASH_TX_BUFFER_SIZE);
	 spiXfer = NULL;*/
//	 b_transferDone = FALSE;
   //  Init_SPI_PinMux();
	 /* Setup SPI controllers */
	 setupMaster();
	 /* Enable SPI controller interrupts */
	 NVIC_EnableIRQ(LPC_SPIFLASHIRQNUM);
	//  NVIC_EnableIRQ(LPC_SPIACCELIRQNUM);

	 Set_Timer(&OFFLINE_DATA_RECORD_TIMER, Get_LogPeriod());



//	 Set_Timer(&OFFLINE_DATA_FLUSH_TIMER, OFFLINE_DATA_FLUSH_TIMEOUT);
}

/* Initializes pin muxing for SPI0 interface -*/
static void Init_SPI_PinMux(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_SWM_MovablePinAssign(SWM_SPI0_SSEL0_IO, 26);
	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 24);
	Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, 25);
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, 15);

	/* initialize SPI1 interface for accelerometer */
/*	Chip_SWM_MovablePinAssign(SWM_SPI1_SCK_IO, 7);
	Chip_SWM_MovablePinAssign(SWM_SPI1_MISO_IO, 6);
	Chip_SWM_MovablePinAssign(SWM_SPI1_MOSI_IO, 1);*/

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/* SPI master select assertion callback function */
static void SPIFlash_MasterAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates the master just asserted the slave select
	   signal */
//	PRINT_K("SPIMasterAssert\r\n");
}

/* SPI master select assertion callback function */
static void SPIAccel_MasterAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates the master just asserted the slave select
	   signal */
//	PRINT_K("SPIMasterAssert\r\n");
}

/* SPI master send data callback function */
static void SPIFlash_MasterSendData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs more data to
	   send. The pMasterXfer->pTXData buffer pointer and transfer size
	   (in items) in pMasterXfer->txCount should be updated. */

	/* If this function sets the pMasterXfer->terminate flag to true,
	   this function won't be called again and the transfer will
	   terminate when the current transmit buffer is complete. */
	//PRINT_K("SPIMasterSendData\r\n");
}
/* SPI master send data callback function */
static void SPIAccel_MasterSendData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs more data to
	   send. The pMasterXfer->pTXData buffer pointer and transfer size
	   (in items) in pMasterXfer->txCount should be updated. */

	/* If this function sets the pMasterXfer->terminate flag to true,
	   this function won't be called again and the transfer will
	   terminate when the current transmit buffer is complete. */
	//PRINT_K("SPIMasterSendData\r\n");
}

/* SPI master receive data callback function */
static void SPIFlash_MasterRecvData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs another receive
	   buffer. The pMasterXfer->pRXData buffer pointer and transfer size
	   (in items) in pMasterXfer->rxCount should be updated. */

//	PRINT_K("SPIMasterRecvData\r\n");
}
/* SPI master receive data callback function */
static void SPIAccel_MasterRecvData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs another receive
	   buffer. The pMasterXfer->pRXData buffer pointer and transfer size
	   (in items) in pMasterXfer->rxCount should be updated. */

//	PRINT_K("SPIMasterRecvData\r\n");
}

/* SPI master select de-assertion callback function */
static void SPIFlash_MasterDeAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates tha master just deasserted the slave select
	   signal */
//	PRINT_K("SPIMasterDeAssert\r\n");
}
/* SPI master select de-assertion callback function */
static void SPIAccel_MasterDeAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates tha master just deasserted the slave select
	   signal */
//	PRINT_K("SPIMasterDeAssert\r\n");
}

/* SPI master transfer done callback */
static void SPIFlash_MasterDone(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates the transfer is complete */
    b_transferDone = TRUE;
}

/* SPI master transfer done callback */
static void SPIAccel_MasterDone(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates the transfer is complete */
  //  b_transferDone = TRUE;
}

/* SPI master driver callbacks */
static const SPIM_CALLBACKS_T spiFlashCallbacks = {
	&SPIFlash_MasterAssert,
	&SPIFlash_MasterSendData,
	&SPIFlash_MasterRecvData,
	&SPIFlash_MasterDeAssert,
	&SPIFlash_MasterDone
};
static const SPIM_CALLBACKS_T spiAccelCallbacks = {
	&SPIAccel_MasterAssert,
	&SPIAccel_MasterSendData,
	&SPIAccel_MasterRecvData,
	&SPIAccel_MasterDeAssert,
	&SPIAccel_MasterDone
};
/* Setup master controller */
void setupMaster(void)
{

	/* Initialize SPI controllers */
	Chip_SPI_Init(LPC_SPIFLASHPORT);
//	Chip_SPI_Init(LPC_SPIACCELPORT);

	/* Call to initialize first SPI controller for mode0, master mode, MSB first */
	Chip_SPI_ConfigureSPI(LPC_SPIFLASHPORT,SPI_MODE_MASTER |	    /* Enable master mode */
						 	 SPI_CLOCK_CPHA0_CPOL0 |	            /* Set Clock polarity to 0 */
							 SPI_CFG_MSB_FIRST_EN |                 /* Enable MSB first option */
							 SPI_CFG_SPOL_LO);	                    /* Chipselect is active low */

	/* Call to initialize second SPI controller for mode0, master mode, MSB first */
//	Chip_SPI_ConfigureSPI(LPC_SPIACCELPORT,SPI_MODE_MASTER |	    /* Enable master mode */
//							 SPI_CLOCK_CPHA0_CPOL0 |	            /* Set Clock polarity to 0 */
//							 SPI_CFG_MSB_FIRST_EN |                 /* Enable MSB first option */
//							 SPI_CFG_SPOL_LO);	                    /* Chipselect is active low */

	/* Setup master clock rate, slave clock doesn't need to be setup */
	Chip_SPIM_SetClockRate(LPC_SPIFLASHPORT, SPIFLASHCLOCKRATE);
//	Chip_SPIM_SetClockRate(LPC_SPIACCELPORT, SPIACCELCLOCKRATE);

	/* Setup master delay (all chip selects) */
/*	masterDelay.PreDelay = 0xD;
	masterDelay.PostDelay = 0xD;
	masterDelay.FrameDelay = 0xD;
	masterDelay.TransferDelay = 0xD;
	Chip_SPIM_DelayConfig(LPC_SPIFLASHPORT, &masterDelay);*/
	//Chip_SPIM_DelayConfig(LPC_SPIACCELPORT, &masterDelay);

	/* For the SPI controller configured in master mode, enable SPI master interrupts
	   for interrupt service. Do not enable SPI_INTENSET_TXDYEN. */
	Chip_SPI_EnableInts(LPC_SPIFLASHPORT,  (SPI_INTENSET_RXDYEN |
											SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
											SPI_INTENSET_SSDEN));

	/*Chip_SPI_EnableInts(LPC_SPIACCELPORT,  (SPI_INTENSET_RXDYEN |
											SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
											SPI_INTENSET_SSDEN));
*/
	/* Setup master transfer callbacks in the transfer descriptor */
	spiFlashXfer.pCB = &spiFlashCallbacks;
//	spiAccelXfer.pCB = &spiAccelCallbacks;
}
/****************************************************************************/
/* Interrupt service routine for SPI Flash transfers                        */
/****************************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) SPI0_IRQHandler(void)
{
	uint32_t ints = Chip_SPI_GetPendingInts(LPC_SPIFLASHPORT);

	/* Handle SPI slave interrupts only */
	if ((ints & (SPI_INTENSET_RXDYEN | SPI_INTENSET_RXOVEN |
				 SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN | SPI_INTENSET_SSDEN)) != 0) {
		/* SPI slave handler */
		Chip_SPIM_XferHandler(LPC_SPIFLASHPORT, &spiFlashXfer);
	}
}
/****************************************************************************/
/* Interrupt service routine for SPI Accelerometer transfers                */
/****************************************************************************/
__attribute__((short_call))
void __attribute__ ((noinline)) SPI1_IRQHandler(void)
{
	IntDefaultHandler();
	/*uint32_t ints = Chip_SPI_GetPendingInts(LPC_SPIACCELPORT);*/

	/* Handle SPI slave interrupts only */
	/*if ((ints & (SPI_INTENSET_RXDYEN | SPI_INTENSET_RXOVEN |
				 SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN | SPI_INTENSET_SSDEN)) != 0) {*/
		/* SPI slave handler */
	/*	Chip_SPIM_XferHandler(LPC_SPIACCELPORT, &spiAccelXfer);
	}*/
}
/*******************************************************************************/
void send_spi_data( LPC_SPI_T *pSPI,
					uint8_t *p_writeBuffer,
				    uint16_t u8_numBytesToBeWritten,
				    uint8_t *p_readBuffer,
				    uint16_t u8_numBytesToBeRead )
{
	/*if(u8_numBytesToBeWritten > SPI_FLASH_TX_BUFFER_SIZE ||
	   u8_numBytesToBeRead > SPI_FLASH_RX_BUFFER_SIZE)
		return;*/

	if(pSPI == LPC_SPIFLASHPORT){
		spiXfer = &spiFlashXfer;
	}

	/*else if(pSPI == LPC_SPIACCELPORT){
		spiXfer = &spiAccelXfer;
	}*/
/*	else*
	{
		PRINT_K("SPI ERROR");
		return;
	}*/

	spiXfer->pTXData8 = p_writeBuffer;	/* Use NULL to send 0x0 */
	spiXfer->pRXData8 = p_readBuffer;
	spiXfer->txCount = u8_numBytesToBeWritten;
	spiXfer->rxCount = u8_numBytesToBeRead;/* Count is in transfer size */
	/* Setup master transfer options - 8 data bits per transfer, EOT, EOF */
	spiXfer->options =
			SPI_TXCTL_FLEN(8) |		/* This must be enabled as a minimum, use 16 data bits */
			//SPI_TXCTL_EOF |			/* Insert a delay between bytes/words as defined by frame delay time */
			0;

	/*Transfer will terminate after current buffer is sent. If terminate is not set, the buffers
	  must be setup by the callbacks		*/
	spiXfer->terminate = true;

	/* Use SPI select 0 */
	spiXfer->sselNum = 0;
	Chip_SPI_FlushFifos(pSPI);
	/* Start master transfer */

	Chip_SPIM_Xfer(pSPI, spiXfer);

	while(!b_transferDone)
	{}
	b_transferDone = FALSE;
}
/************************************************************************/
/* FUNCTION: Jedec_ID_Read						                        */
/*									                                    */
/* This function reads the manufacturer's ID (BFh), memory type (25h)  */
/* and device ID (41h).  It will use 9Fh as the JEDEC ID command.    	*/
/*                                                                      */
/*  Returns:				 				                                */
/*	IDs_Read:ID1(Manufacture's ID = BFh, Memory Type (25h),             */
/*  and Device ID (80h)				                                	*/
/*									                                    */
/************************************************************************/
bool SST25_Jedec_ID_Read()
{
	uint8_t spiRxBuffer[4];

	flashTransferTXBuffer[0] = SST25_JEDEC_ID; /* send JEDEC ID command (9Fh) */
	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer,4 , spiRxBuffer, 4);

	if((spiRxBuffer[1] == 0x01) && (spiRxBuffer[2] == 0x40) && (spiRxBuffer[3] == 0x14)){

		//PRINT_K("Spansion S25FL208\n");
		return TRUE;
	}

	else{
	//	PRINT_K("ERROR\n");
		return FALSE;
	}

}
/************************************************************************/
/* PROCEDURE: WREN						                            	*/
/*																		*/
/* This procedure enables the Write Enable Latch.  It can also be used 	*/
/* to Enables Write Status Register.									*/
/* Returns:																*/
/*		Nothing															*/
/************************************************************************/
void SST25_WriteEnable()
{
	uint8_t spiRxBuffer[2];

	//PRINT_K("SST25_Write_Enable...");
	flashTransferTXBuffer[0] = SST25_WREN; /* send WREN command (0x06) */
	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 1 , spiRxBuffer, 1);
	//	PRINT_K("Done...\r\n");
	wait_spi_busy_timeout(FLASH_WRITE_SR_TIME);

}
/*************************************************************************/
void SST25_WriteDisable()
{
	uint8_t spiRxBuffer[2];

	flashTransferTXBuffer[0] = SST25_WRDI; /* send WREN command (0x06) */
	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 1 , spiRxBuffer, 1);
}
/*************************************************************************/
void SST25_Write(uint32_t address, uint16_t data)
{
	//PRINT_K("\r\nWriting...");
		//flashTransferTXBuffer[0] = SST25_AAI; /* send AAI command (ADh) */
	/*	flashTransferTXBuffer[1] = WORD32_BYTE2(address);
		flashTransferTXBuffer[2] = WORD32_BYTE1(address);
		flashTransferTXBuffer[3] = WORD32_BYTE0(address);
		flashTransferTXBuffer[4] = HIGHBYTE(data);
		flashTransferTXBuffer[5] = LOWBYTE(data);

		send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 6, flashTransferRXBuffer, 6);
		PRINT_K("Done\r\n");*/
}
//****************************************************************************/
SPI_RESULT_T SST25_Read(uint32_t address, uint16_t length, uint8_t *buffer)
{
	int i;
	uint8_t spiRxBuffer[512];

	flashTransferTXBuffer[0] = SST25_READ;       /* Read byte command (03h) */
	flashTransferTXBuffer[1] = WORD32_BYTE2(address);
	flashTransferTXBuffer[2] = WORD32_BYTE1(address);
	flashTransferTXBuffer[3] = WORD32_BYTE0(address);

	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, length+ 4, spiRxBuffer, length+ 4);

	for(i = 4; i < length + 4; i++)
		buffer[i - 4] = spiRxBuffer[i];

	return SPI_WRITE_SUCCESS;
}
//****************************************************************************/
SPI_RESULT_T MMA7455_Read(uint32_t address, uint16_t length, uint8_t *buffer)
{
	uint8_t *tempBuffer;
    char buf[8];
	//tempBuffer = malloc(length + 1);

	/*if(tempBuffer == NULL){
			PRINT_K("Panic... Cannot allocate memory\r\n");
			while(1);
			return SPI_ERROR_ALLOCATING_MEMORY;
	}*/

	flashTransferTXBuffer[0] = 0x38;
	flashTransferTXBuffer[1] = 0x01;

	send_spi_data(LPC_SPIACCELPORT, flashTransferTXBuffer, 2, tempBuffer, 2);
	memcpy(buffer, tempBuffer+1, length);
//	PRINT_K("Reading accelo\r\n");
//	itoa(tempBuffer[0], buf ,16);
	PRINT_K(buf);
	//itoa(tempBuffer[1], buf ,16);
	PRINT_K(buf);
	//free(tempBuffer);
	return SPI_WRITE_SUCCESS;
}
/********************************************************************/
void SST25_ChipErase()
{
	uint8_t spiRxBuffer[2];

	SST25_WriteSR(0x0);         /* disable all block protection */
	SST25_WriteEnable();

	PRINT_K("\nErasing...");

	flashTransferTXBuffer[0] = SST25_CE;       /* Erase chip command (C7h) */
	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 1, spiRxBuffer, 1);
	wait_spi_busy_timeout(FLASH_CHIP_ERASE_TIME);
	PRINT_K("Done...\n");


}
void ReadStatusReg(char *msg)
{
	uint8_t sr;
	PRINT_K(msg);
	sr = SST25_ReadSR();
	Print_Val(msg, sr);

//	SST25_Read(TEST_ADDRESS, sizeof(testData),(uint8_t *)&testData);

}
/*******************************************************************/
uint8_t SST25_ReadSR()
{
	uint8_t spiRxBuffer[4];

	//PRINT_K("Reading status register...");
	flashTransferTXBuffer[0] = SST25_RDSR;
	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 2, spiRxBuffer, 2);
	return spiRxBuffer[1];
	//	PRINT_K("Done\r\n");
}
/*************************************************************************/
void SST25_WriteByte(uint32_t address, uint8_t data)
{
	uint8_t spiRxBuffer[5];

	SST25_WriteEnable();

	flashTransferTXBuffer[0] = SST25_BP; /* Byte program command (0x02) */
	flashTransferTXBuffer[1] = WORD32_BYTE2(address);
	flashTransferTXBuffer[2] = WORD32_BYTE1(address);
	flashTransferTXBuffer[3] = WORD32_BYTE0(address);
	flashTransferTXBuffer[4] = data;

	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 5, spiRxBuffer, 5);
	wait_spi_busy_timeout(FLASH_WRITE_BYTE_TIMEOUT);
}
/************************************************************************/
/* Function: SST25_EnableWSR							                */
/*									                                    */
/* This function Enables Write Status Register.  			            */
/* Input:								                                */
/*		 None							                                */
/*									                                    */
/* Returns:								                                */
/*		Nothing							                                */
/************************************************************************/
void SST25_EnableWSR()
{
	uint8_t spiRxBuffer[2];

	flashTransferTXBuffer[0]= SST25_EWSR;
	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 1, spiRxBuffer, 1);
	wait_spi_busy_timeout(FLASH_WRITE_SR_TIME);
}
/************************************************************************/
/* Function: SST25_WriteSR						                        */
/*								                                    	*/
/* This function writes a byte to the Status Register.					*/
/*																		*/
/* Input:																*/
/*		byte															*/
/*																		*/
/* Returns:																*/
/*		Nothing															*/
/************************************************************************/
void SST25_WriteSR(uint8_t data)
{

	uint8_t spiRxBuffer[2];
	SST25_WriteEnable();

	SST25_EnableWSR();

	flashTransferTXBuffer[0]= SST25_WRSR;
	flashTransferTXBuffer[1]= data;

	send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 2, spiRxBuffer, 2);
	wait_spi_busy_timeout(FLASH_WRITE_SR_TIME);
}
/*************************************************************************/
/* Function SST25_ReadBUSY
 *
 * Checks BUSY flag of Status Register
 *
 * Inputs: None
 *
 * Returns bool status of busy bit
 */
/*************************************************************************/
 bool SST25_ReadBUSY()
{
	if(SST25_ReadSR() & FLASH_BUSY)
	 	return TRUE;
	 else
	 	return FALSE;
}
/*************************************************************************/
/* Function SST25_EraseSector
*
* Checks Erase a sector
*
* Inputs: Sector address to be erased
*
* Returns bool status of busy bit
**************************************************************************/
 void SST25_EraseSector(uint32_t sector_no)
 {
	 uint8_t spiRxBuffer[4];

	 uint32_t address = sector_no * SST25_SECTOR_SIZE;

	 SST25_WriteEnable();

	// PRINT_K("\r\nErasing sector...");

	 flashTransferTXBuffer[0]= SST25_SE;		/* erase 4K sector command	*/
	 flashTransferTXBuffer[1] = WORD32_BYTE2(address);
	 flashTransferTXBuffer[2] = WORD32_BYTE1(address);
	 flashTransferTXBuffer[3] = WORD32_BYTE0(address);

	 send_spi_data(LPC_SPIFLASHPORT, flashTransferTXBuffer, 4, spiRxBuffer, 4);
	 wait_spi_busy_timeout(FLASH_SECTOR_ERASE_TIME);

 }
 /**************************************************************************/
 uint32_t SST25_WriteArray(uint8_t *array, uint16_t length, uint32_t u32_flashAddress)
{
	uint16_t i;

	for(i = 0; i< length; i++){
		if((u32_flashAddress & SST25_FLASH_SECTOR_MASK) == 0)
			SST25_EraseSector(u32_flashAddress/SST25_SECTOR_SIZE);

		SST25_WriteByte(u32_flashAddress, array[i]);
		u32_flashAddress++;
	}
	return u32_flashAddress;
}
/****************************************************************/
bool SST25_IsEmpty()
{
	uint32_t buffer, i;

		for(i = 0; i < SST25_SECTOR_COUNT; i++){
			SST25_Read(SST25_SECTOR_SIZE * i, sizeof(uint32_t), (uint8_t *)&buffer);
			if(buffer != 0xFFFFFFFF){
			/*	PRINT_INT(i);
				PRINT_K(" sector read\r\n");*/
				return FALSE;
			}
		}
	/*	PRINT_INT(i);
		PRINT_K(" sector read\r\n");*/
		return TRUE;
}
/*******************************************************************/
void UpdateOffLineDataWriteAddress(uint32_t address)
{
	SST25_WriteSR(0x00);

	//Print_Val("\nWriting to 0x",address);
	//PRINT_K(" to Address\r\n");
	SST25_WriteArray((uint8_t *)&address, sizeof(address), OFFLINE_DATA_WRITE_ADDRESS);
	SST25_WriteSR(SST25_BP1);
}
/*******************************************************************/
void UpdateOffLineDataReadAddress()
{

	SST25_WriteSR(0x00);

//	Print_Val("\nWriting 0x",offLineData.u32_offLineDataReadAddress);
//	PRINT_K(" to FLASH Read Address\r\n");
	//Print_Val("to address 0x",OFFLINE_DATA_READ_ADDRESS);
//	PRINT_K("...");

	SST25_WriteArray((uint8_t *)&offLineData.u32_offLineDataReadAddress,
					 sizeof(offLineData.u32_offLineDataReadAddress),
					 OFFLINE_DATA_READ_ADDRESS);

	offLineData.u32_offLineDataReadAddressPersisted = offLineData.u32_offLineDataReadAddress;

	SST25_WriteSR(SST25_BP1);
}
/**********************************************************************/
void Trio_SPITask()
{
	GGA_MESSAGE_T gga_info;
	GSA_MESSAGE_T gsa_info;
	EVENT_INFO_T  event_info;
	uint32_t i_msgLen;
	uint8_t c_TMessageBuffer[MAX_LOG_ENTRY_SIZE];

	Get_GGAInfo(&gga_info);
	Get_GSAInfo(&gsa_info);

	if(mn_timer_expired(&OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMER)){
		if(offLineData.u32_offLineDataReadAddressPersisted != offLineData.u32_offLineDataReadAddress)
			UpdateOffLineDataReadAddress();

		Set_Timer(&OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMER, OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMEOUT);
	}

	if(!GetServerConnectionStatus()){
		if(mn_timer_expired(&OFFLINE_DATA_RECORD_TIMER)){
			if((gsa_info.mode2 == NMEA_GPGSA_MODE2_3D) && (gga_info.position_fix)){
				PRINT_K("\nLOGING GPS DATA\n");
				memset(c_TMessageBuffer, 0, MAX_LOG_ENTRY_SIZE);
				i_msgLen = Trio_PrepareTMessage((char *)c_TMessageBuffer, FALSE);
				AddOffLineRecord(c_TMessageBuffer, i_msgLen);
			}
		Set_Timer(&OFFLINE_DATA_RECORD_TIMER,  Get_LogPeriod());
		}
		if(Get_EventStatus(&event_info)){
			PRINT_K("\nLOGING EVENT\n");
			memset(c_TMessageBuffer, 0, MAX_LOG_ENTRY_SIZE);
			i_msgLen = Trio_PrepareTMessage((char *)c_TMessageBuffer, FALSE);
			AddOffLineRecord(c_TMessageBuffer, i_msgLen);
		}
	}
	else{
		if(offLineData.bufferedBytes > 0)
			FlushOffLineDataBuffer();
	}
}
/***************************************************************************/
void ReloadLogTimer()
{
	Set_Timer(&OFFLINE_DATA_RECORD_TIMER, Get_LogPeriod());
}
/***************************************************************************/
TIMER_TICK_T  Get_LogPeriod()
{
	/*if(Get_IgnitionStatus())
		return OFFLINE_IGNITED_LOG_PERIOD;
	else
		return OFFLINE_NOT_IGNITED_LOG_PERIOD;*/
	TIMER_TICK_T period;

	if(Get_IgnitionPinStatus())
		period =  MAX(Get_DataSendPeriod(), OFFLINE_IGNITED_LOG_PERIOD);
	else
		period =  MAX(Get_DataSendPeriod(), OFFLINE_NOT_IGNITED_LOG_PERIOD);


/*	PRINT_K("\nSetting log period to ");
	PRINT_INT(period/100);
	PRINT_K("secs\n");*/
	return period;
}
/***************************************************************************/
void AddOffLineRecord(uint8_t *p_msgPtr, uint32_t i_msgLen)
{
	/* check if we have enough space in cache buffer */
	if(offLineData.bufferedBytes + i_msgLen + 6 > OFFLINE_DATA_CACHE_SIZE)
		FlushOffLineDataBuffer();

		AddOfflineDataToCache(p_msgPtr, i_msgLen);
}
/************************************************************************************/
void AddOfflineDataToCache(uint8_t *p_msgPtr, uint16_t i_msgLen)
{
	uint16_t u16_crc;
	uint8_t *p_bufPtr;
	char  strCRC[8];
	uint8_t *p_msgStart;

    memset(strCRC, 0, sizeof(strCRC));

	p_bufPtr = &offLineData.buffer[offLineData.bufferedBytes];
	p_msgStart = p_bufPtr;

	*(p_bufPtr++) = CHAR_STX;      /* start log message */

	u16_crc = crc16(p_msgPtr, i_msgLen);
	memcpy(p_bufPtr, p_msgPtr, i_msgLen);
	p_bufPtr+= i_msgLen;
	Hex2Str(strCRC, u16_crc);
	memcpy(p_bufPtr, &strCRC[4], 4);
	p_bufPtr+= 4;

	*(p_bufPtr++) = CHAR_ETX;     /* end log message */
	*p_bufPtr = '\0';             /* stringize message*/

	PRINT_K(p_msgStart);
	PRINT_INT(p_bufPtr- p_msgStart);
	PRINT_K(" bytes buffered\n");

	offLineData.bufferedBytes+= (p_bufPtr- p_msgStart);
//	PRINT_INT(offLineData.bufferedBytes);

}
/************************************************************************************/
#define  LOG_BLOCK_READ_TIMEOUT      5

int32_t Get_OffLineDataFromFlash(char *buffer)
{
	TIMER_INFO_T spiTimer;
	LOG_ENTRY_INFO_T log_entry;

		Set_Timer(&spiTimer, LOG_BLOCK_READ_TIMEOUT);
		while(!mn_timer_expired(&spiTimer)){
			if(Get_LogEntry(&log_entry) < 0)   /* no new log entry exist*/
				return 0;
			if(log_entry.b_validEntry){
				memset(buffer, 0, MAX_LOG_ENTRY_SIZE);
				strcpy(buffer, log_entry.logBuffer);
				return strlen(buffer);
			}
		}
		return 0;
	//PRINT_K("\r\nNo off-line data\r\n");
}
/*********************************************************************/
uint16_t Get_ReadBlockSize()
{
/*	PRINT_K("\r\n******************\r\n");
	Print_Val("\r\noffLineDataReadAddress",offLineData.u32_offLineDataReadAddress);
	Print_Val("\r\noffLineDataWriteAddress",offLineData.u32_offLineDataWriteAddress);
	PRINT_K("\r\n*****************\r\n");*/
	if(offLineData.u32_offLineDataWriteAddress > offLineData.u32_offLineDataReadAddress){
		if((offLineData.u32_offLineDataReadAddress + MAX_LOG_ENTRY_SIZE) > offLineData.u32_offLineDataWriteAddress)
			return offLineData.u32_offLineDataWriteAddress - offLineData.u32_offLineDataReadAddress;
		else
			return MAX_LOG_ENTRY_SIZE;
	}
	else{
		if((offLineData.u32_offLineDataReadAddress + MAX_LOG_ENTRY_SIZE) >= LOG_UPPER_BOUNDARY_ADDRESS)
			return LOG_UPPER_BOUNDARY_SECTOR- offLineData.u32_offLineDataReadAddress;
		else
			return MAX_LOG_ENTRY_SIZE;
	}
}
/************************************************************************************/
static int Get_LogEntry(LOG_ENTRY_INFO_T *log_info)
{
	uint8_t *p_msgStart, *p_msgEnd, *p_nextStx;
	uint16_t i_blockSize;
	uint16_t u16_crcCalc;
	char strCRCRead[5];
	char strCRCCalc[9];
	uint8_t buffer[MAX_LOG_ENTRY_SIZE];

	log_info->b_validEntry = FALSE;

	if(offLineData.u32_offLineDataWriteAddress == offLineData.u32_offLineDataReadAddress)
		return -1;

	memset(log_info->logBuffer, 0, MAX_LOG_ENTRY_SIZE);
	memset(strCRCCalc, 0, sizeof(strCRCCalc));

	i_blockSize  = Get_ReadBlockSize();

	if(i_blockSize >= MAX_LOG_ENTRY_SIZE)
		i_blockSize = MAX_LOG_ENTRY_SIZE;

	Print_Val("\nReading 0x",offLineData.u32_offLineDataReadAddress);
/*	PRINT_K(" ");
	PRINT_INT(i_blockSize);
	PRINT_K(" bytes\r\n");*/

	SST25_Read(offLineData.u32_offLineDataReadAddress, i_blockSize, buffer);

    /* get pointers to message start stop chars*/
	p_msgStart = (uint8_t *)strchr((char *)buffer, CHAR_STX);
	p_msgEnd = (uint8_t *)strchr((char *)buffer, CHAR_ETX);

	/* both [ETX] and [STX] found but in reverse order */
	if(p_msgStart != NULL && p_msgEnd != NULL){
		if(p_msgStart > p_msgEnd){
			offLineData.u32_offLineDataReadAddress += (p_msgStart - buffer);
			PRINT_K("\nErr 1");
		//	PRINT_K(p_msgStart);
		}

		else{ /* both [ETX] and [STX] found in correct order */
			*p_msgEnd = '\0';
			strcpy((char *)strCRCRead, (char *)p_msgEnd - 4);    /* extract CRC bytes in current record */
			*(p_msgEnd - 4) = '\0';

			u16_crcCalc = crc16(p_msgStart + 1, strlen((char *)(p_msgStart + 1)));
			Hex2Str(strCRCCalc, u16_crcCalc);
			if(strcmp(strCRCRead, strCRCCalc + 4) == 0){   /* compare CRC strings*/
				strcpy(log_info->logBuffer, (char *)(p_msgStart + 1));
				log_info->b_validEntry = TRUE;
				PRINT_K("\nCorrect");
			}
			else{
				PRINT_K("\nCRC Err");
				log_info->b_validEntry = FALSE;
			}
			offLineData.u32_offLineDataReadAddress += (p_msgEnd - p_msgStart + 1);
		}
	}

	/* both [ETX] and [STX] not found */
	else if(p_msgStart == NULL && p_msgEnd == NULL){
		PRINT_K("\nErr 2");
		buffer[MAX_LOG_ENTRY_SIZE - 1] = '\0';
	//	PRINT_K(buffer);
		offLineData.u32_offLineDataReadAddress +=  i_blockSize;
	}

	/* [ETX] found  no [STX] found */
	else if(p_msgStart == NULL && p_msgEnd != NULL){
		*p_msgEnd = '\0';
	//	PRINT_K(buffer);
		offLineData.u32_offLineDataReadAddress +=  (p_msgEnd - buffer + 1) ;
	}

	/* [STX] found but no [ETX] found */
	else if(p_msgStart != NULL && p_msgEnd == NULL){
		PRINT_K("\nErr 4");
		p_nextStx = (uint8_t *)strchr((char *)(p_msgStart + 1), CHAR_STX);
		if(p_nextStx != NULL){
			*(p_nextStx - 1) ='\0';
		//	PRINT_K(p_msgStart);
			offLineData.u32_offLineDataReadAddress += (p_nextStx - buffer);
		}
		else
			offLineData.u32_offLineDataReadAddress += i_blockSize;
	}

	if(offLineData.u32_offLineDataReadAddress >= LOG_UPPER_BOUNDARY_ADDRESS)
		offLineData.u32_offLineDataReadAddress = LOG_LOWER_BOUNDARY_ADDRESS;

	if(offLineData.u32_offLineDataWriteAddress == offLineData.u32_offLineDataReadAddress)
		UpdateOffLineDataReadAddress();
	return 1;
}
/************************************************************************************/
void Init_SPI_Cache()
{

	memset(&offLineData, 0, sizeof(offLineData));


	/*SST25_Read(TEST_ADDRESS,sizeof(testData), (uint8_t *)&testData);
	Print_Val("\nTest Data 0x", testData);*/

	SST25_Read(OFFLINE_DATA_WRITE_ADDRESS,
			   sizeof(offLineData.u32_offLineDataWriteAddress),
			   (uint8_t *)&offLineData.u32_offLineDataWriteAddress);

	/* check against invalid write address value */
	if(offLineData.u32_offLineDataWriteAddress  >= LOG_UPPER_BOUNDARY_ADDRESS)
		offLineData.u32_offLineDataWriteAddress =0;

	Print_Val("\nLog Write Addr 0x",offLineData.u32_offLineDataWriteAddress);

	SST25_Read(OFFLINE_DATA_READ_ADDRESS,
			   sizeof(offLineData.u32_offLineDataReadAddress),
			   (uint8_t *)&(offLineData.u32_offLineDataReadAddress));

	/* check against invalid read address value */
	if(offLineData.u32_offLineDataReadAddress  >= LOG_UPPER_BOUNDARY_ADDRESS)
			offLineData.u32_offLineDataReadAddress =SST25_SECTOR_SIZE;

	offLineData.u32_offLineDataReadAddressPersisted = offLineData.u32_offLineDataReadAddress;


	Print_Val("\nLog Read Addr 0x",offLineData.u32_offLineDataReadAddress);

	SST25_WriteSR(SST25_BP1);

	Set_Timer(&OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMER, OFFLINE_DATA_READ_ADDRESS_UPDATE_TIMEOUT);


}
/*************************************************************************************/
void FlushOffLineDataBuffer2()
{
	uint32_t u32_nextWriteAddress = offLineData.u32_offLineDataWriteAddress;


	if((Get_EndWriteSectorNumber() == LOG_UPPER_BOUNDARY_SECTOR) &&
	   (Get_CurrentReadSectorNumber() == LOG_LOWER_BOUNDARY_SECTOR) &&
	   (Get_CurrentWriteSectorNumber() == (LOG_UPPER_BOUNDARY_SECTOR - 1))){
		PRINT_K("CASE 1\n");
		offLineData.u32_offLineDataWriteAddress = LOG_LOWER_BOUNDARY_ADDRESS;
		u32_nextWriteAddress = offLineData.bufferedBytes;
		offLineData.u32_offLineDataReadAddress = SST25_SECTOR_SIZE;
		UpdateOffLineDataReadAddress();
		//PRINT_K("\r\nFLUSH 1\r\n");
	}
	else if((Get_EndWriteSectorNumber() == LOG_UPPER_BOUNDARY_SECTOR) &&
			(Get_CurrentReadSectorNumber() == LOG_UPPER_BOUNDARY_SECTOR)){
	    	PRINT_K("CASE 2\n");
			offLineData.u32_offLineDataReadAddress = LOG_LOWER_BOUNDARY_ADDRESS;
			u32_nextWriteAddress += offLineData.bufferedBytes;
		//	PRINT_K("\r\nFLUSH 2\r\n");
	}
	else{
		if((Get_EndWriteSectorNumber() > Get_CurrentWriteSectorNumber()) &&
		   Get_EndWriteSectorNumber() == Get_CurrentReadSectorNumber()){
			PRINT_K("CASE 3\n");
			offLineData.u32_offLineDataReadAddress = (Get_CurrentReadSectorNumber() + 1) * SST25_SECTOR_SIZE;

			if(offLineData.u32_offLineDataReadAddress >= LOG_UPPER_BOUNDARY_ADDRESS){
				PRINT_K("CASE 4\n");
				offLineData.u32_offLineDataReadAddress = LOG_LOWER_BOUNDARY_ADDRESS;
			}

			UpdateOffLineDataReadAddress();
			//PRINT_K("\r\nFLUSH 3\r\n");
		}
		u32_nextWriteAddress += offLineData.bufferedBytes;
	}
	Print_Val("\nWriting ",offLineData.bufferedBytes);

	SST25_WriteArray(offLineData.buffer, offLineData.bufferedBytes, offLineData.u32_offLineDataWriteAddress);
	UpdateOffLineDataWriteAddress(u32_nextWriteAddress);
	offLineData.u32_offLineDataWriteAddress = u32_nextWriteAddress;

/*	PRINT_K("\r\n********************************************\r\n");*/
	Print_Val("\nLog Read Address 0x",offLineData.u32_offLineDataReadAddress);
	Print_Val("\nLog Write Address 0x",offLineData.u32_offLineDataWriteAddress);
/*	PRINT_K("\r\n********************************************\r\n");*/

	offLineData.bufferedBytes = 0;
	memset(offLineData.buffer, 0, OFFLINE_DATA_CACHE_SIZE);
}

void FlushOffLineDataBuffer()
{
	uint32_t u32_nextWriteAddress = offLineData.u32_offLineDataWriteAddress;

	if((Get_EndWriteSectorNumber() == LOG_LOWER_BOUNDARY_SECTOR) &&
	   (Get_CurrentWriteSectorNumber() == (LOG_UPPER_BOUNDARY_SECTOR - 1))){
	//	PRINT_K("CASE 1\n");
		offLineData.u32_offLineDataWriteAddress = LOG_LOWER_BOUNDARY_ADDRESS;
		u32_nextWriteAddress = offLineData.bufferedBytes;
		 if(Get_CurrentReadSectorNumber() == LOG_LOWER_BOUNDARY_SECTOR){
			 offLineData.u32_offLineDataReadAddress = SST25_SECTOR_SIZE;
			 UpdateOffLineDataReadAddress();
		 }
	}
	else if((Get_EndWriteSectorNumber() == LOG_UPPER_BOUNDARY_SECTOR - 1) &&
			(Get_CurrentReadSectorNumber() == LOG_UPPER_BOUNDARY_SECTOR - 1)){
	    //	PRINT_K("CASE 2\n");
			offLineData.u32_offLineDataReadAddress = LOG_LOWER_BOUNDARY_ADDRESS;
			u32_nextWriteAddress += offLineData.bufferedBytes;
	}
	else{
		if((Get_EndWriteSectorNumber() > Get_CurrentWriteSectorNumber()) &&
		   Get_EndWriteSectorNumber() == Get_CurrentReadSectorNumber()){
		//	PRINT_K("CASE 3\n");
			offLineData.u32_offLineDataReadAddress = (Get_CurrentReadSectorNumber() + 1) * SST25_SECTOR_SIZE;

		/*	if(offLineData.u32_offLineDataReadAddress >= LOG_UPPER_BOUNDARY_ADDRESS){
				PRINT_K("CASE 4\n");
				offLineData.u32_offLineDataReadAddress = LOG_LOWER_BOUNDARY_ADDRESS;
			}*/

			UpdateOffLineDataReadAddress();
			//PRINT_K("\r\nFLUSH 3\r\n");
		}
		u32_nextWriteAddress += offLineData.bufferedBytes;
	}
	/*PRINT_K("Writing ");
	PRINT_INT(offLineData.bufferedBytes);
	PRINT_K(" bytes\n");*/

	SST25_WriteArray(offLineData.buffer, offLineData.bufferedBytes, offLineData.u32_offLineDataWriteAddress);
	UpdateOffLineDataWriteAddress(u32_nextWriteAddress);
	offLineData.u32_offLineDataWriteAddress = u32_nextWriteAddress;

/*	PRINT_K("\r\n********************************************\r\n");*/
	Print_Val("\nLog Read Address 0x",offLineData.u32_offLineDataReadAddress);
	Print_Val("\nLog Write Address 0x",offLineData.u32_offLineDataWriteAddress);
/*	PRINT_K("\r\n********************************************\r\n");*/

	offLineData.bufferedBytes = 0;
	memset(offLineData.buffer, 0, OFFLINE_DATA_CACHE_SIZE);
}
/************************************************************************/
uint16_t Get_CurrentReadSectorNumber()
{
	 uint32_t result;
	return (uint16_t)(offLineData.u32_offLineDataReadAddress / SST25_SECTOR_SIZE);

	/* PRINT_K("\nCurrentReadSectorNumber ");
	 PRINT_INT(result);
	PRINT_K("\n");
	return result;*/
}
/*************************************************************************/
uint16_t Get_CurrentWriteSectorNumber()
{

	 return (uint16_t)(offLineData.u32_offLineDataWriteAddress / SST25_SECTOR_SIZE);

	/* PRINT_K("\nCurrentWriteSectorNumber ");
	 PRINT_INT(result);
	 PRINT_K("\n");*/

}
/*************************************************************************/
uint16_t Get_EndWriteSectorNumber()
{
	uint32_t result;

	//Print_Val("\nDest Address:", offLineData.u32_offLineDataWriteAddress + offLineData.bufferedBytes);

	if(offLineData.u32_offLineDataWriteAddress + offLineData.bufferedBytes >= LOG_UPPER_BOUNDARY_ADDRESS)
	  result = LOG_LOWER_BOUNDARY_ADDRESS / SST25_SECTOR_SIZE;
	else
	  result =  (offLineData.u32_offLineDataWriteAddress + offLineData.bufferedBytes) / SST25_SECTOR_SIZE;

	/*PRINT_K("\nEndWriteSectorNumber ");
	PRINT_INT(result);
	PRINT_K("\n");*/
	return result;
}
/**************************************************************************/
bool Get_FlashHealtStatus()
{
	return SST25_Jedec_ID_Read();
}
/**************************************************************************/
void wait_spi_busy_timeout(TIMER_TICK_T timeout)
{
	TIMER_INFO_T spiTimer;

	Set_Timer(&spiTimer, timeout);
	while(SST25_ReadBUSY() && !mn_timer_expired(&spiTimer));
}
