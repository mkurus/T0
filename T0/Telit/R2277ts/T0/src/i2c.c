#include "board.h"
#include "i2c.h"
#include "bootloader.h"
#include "timer.h"
#include "bootloader.h"
#include "utils.h"
#include <string.h>
#include "MMA8652.h"

static void cbI2CComplete(uint32_t err_code, uint32_t n);
static void I2cReceiveComplete(uint32_t err_code, uint32_t n);
static void setupI2CMaster();

/* file local variables */
static volatile int intErrCode;
static volatile uint32_t ticks;
bool volatile b_i2ctransferComplete;
static I2C_HANDLE_T *i2cHandleMaster;
/* Use a buffer size larger than the expected return value of
   i2c_get_mem_size() for the static I2C handle type */
static uint32_t i2cMasterHandleMEM[0x20];
/* 100kbps I2C bit-rate */
#define I2C_BITRATE       (100000)

I2C_PARAM_T param;

/***********************************************************************/
void Trio_I2C_Init()
{
	/* Allocate I2C handle, setup I2C rate, and initialize I2C clocking */
	   setupI2CMaster();
	/* Enable the interrupt for the I2C */
	   NVIC_EnableIRQ(I2C1_IRQn);
}
/*************************************************************************/
/* Setup I2C handle and parameters */
static void setupI2CMaster()
{
	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not
		 do this */
	Chip_I2C_Init(ACC_I2C_IFACE);

	/* Perform a sanity check on the storage allocation */
	if (LPC_I2CD_API->i2c_get_mem_size() > sizeof(i2cMasterHandleMEM))
		PRINT_K("Error 1\n");

	/* Setup the I2C handle */
	i2cHandleMaster = LPC_I2CD_API->i2c_setup(LPC_I2C1_BASE, i2cMasterHandleMEM);
	if (i2cHandleMaster == NULL)
		PRINT_K("Error 2\n");

		/* Set I2C bitrate */
	if (LPC_I2CD_API->i2c_set_bitrate(i2cHandleMaster, Chip_Clock_GetSystemClockRate(),
										  I2C_BITRATE) != LPC_OK)
		PRINT_K("Error 3\n");
}
/***************************************************************************/
/* Master transmit in interrupt mode */
void sendI2CMaster(uint8_t *sendBuffer, uint16_t sendLen,  bool stop)
{
	//uint8_t SendData[10];
	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;

	/* 7-bit address */
//	sendData[index++] = (uint8_t) AddressI2C;
//	sendData[index++] = (uint8_t) regAddr;		/* I2C device regAddr */
//	sendData[index++] = (uint8_t) (AddressI2C + 1);

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send    = sendLen;
	param.buffer_ptr_send   = sendBuffer;
	param.num_bytes_rec     = NULL;
	param.buffer_ptr_rec    = 0;
	param.stop_flag         = stop;
	param.func_pt           = cbI2CComplete;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master write transfer */
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_transmit_intr(i2cHandleMaster, &param, &result);

//	error_code = LPC_I2CD_API->i2c_master_receive_intr(i2cHandleMaster, &param, &result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		/* Likely cause is NAK */
	/*	Print_Val("i2c_master_transmit error code : 0x", error_code);*/
		//errorI2C();
	}

	/* Note results are only valid when there are no errors */
}
/***********************************************************************************/
/* Master receive in interrupt mode */

void readI2CMaster(uint8_t *buffer, uint16_t length, bool stop)
{

	I2C_PARAM_T param;
	I2C_RESULT_T result;
	ErrorCode_t error_code;
	int index = 0;
	memset(buffer+5, 0x55, 5);

	/* 7-bit address */
/*	sendData[0] = (uint8_t) (AddressI2C + 1);
	recvData[0] = (uint8_t) (AddressI2C + 1);*/
	/* Setup I2C paameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 (read) - ack
	   value 2 read) - ack - stop */
	param.num_bytes_send    = 1;
	param.num_bytes_rec     = length + 5;
	param.buffer_ptr_rec    = buffer;
	param.stop_flag         = stop;
	param.func_pt           = I2cReceiveComplete;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master read transfer */
	intErrCode = -1;

	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_receive_intr(i2cHandleMaster, &param, &result);

	/* Sleep until transfer is complete, but allow IRQ to wake system
	   to handle I2C IRQ */
	while (intErrCode == -1) {
		__WFI();
	}

	/* Completed without erors? */
	if (error_code != LPC_OK)
		/* Likely cause is NAK */
		PRINT_K("Error 4\n");


}
/* Function not used in this example, provided for reference */
/* Master transmit/receive in interrupt mode */
void SendReadI2CMaster(uint8_t i2c_addr, uint8_t reg, uint8_t * i2c_buffer, uint16_t sendLen, uint16_t recvLen)
{


	I2C_RESULT_T result;
	ErrorCode_t error_code;

	/* 7-bit address */
	i2c_buffer[0] = (i2c_addr << 1) | 1;
	i2c_buffer[1] = reg;

	 /* Setup parameters for transfer */
	param.num_bytes_send  = sendLen;
	param.num_bytes_rec   = recvLen;
	param.buffer_ptr_send = param.buffer_ptr_rec = i2c_buffer;
	param.stop_flag       = 1;
    param.func_pt         = I2cReceiveComplete;

	/* Set timeout */
	LPC_I2CD_API->i2c_set_timeout(i2cHandleMaster, 100000);

	/* Do master read transfer */
	intErrCode = -1;

	b_i2ctransferComplete = FALSE;
	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_tx_rx_intr(i2cHandleMaster, &param, &result);


	/* Cast saved error code from callback */
	error_code = (ErrorCode_t) intErrCode;

	/* Completed without erors? */
	if (error_code != LPC_OK) {
		/* Likely cause is NAK */
		Print_Val("i2c_master_tx_rx error code : 0x", error_code);
		//errorI2C();
	}
}

/****************************************************************************/
/* Handle I2C interrupt by calling I2CM interrupt transfer handler          */
/* return	Nothing                                                         */
/****************************************************************************/
 void I2C1_InterruptHandler()
{
	 /* Call I2C ISR function in ROM with the I2C handle */
	 LPC_I2CD_API->i2c_isr_handler(i2cHandleMaster);
}
 /****************************************************************************/
  __attribute__((short_call))
 void __attribute__ ((noinline)) I2C0_IRQHandler(void)
 {
	  IntDefaultHandler();
 }
/****************************************************************************/
 /* I2C interrupt callback, called on completion of I2C operation when in
    interrupt mode. Called in interrupt context. */
 static void cbI2CComplete(uint32_t err_code, uint32_t n)
 {
 	intErrCode = (int) err_code;
 	Print_Val("i2c_master_tx error code : 0x", err_code);
 	PRINT_K("\nTransmit Complete\r\n");

 	 Print_Val("\nValue:",n);
 }
 /****************************************************************************/
 static void I2cReceiveComplete(uint32_t err_code, uint32_t n)
 {
	Print_Val("i2c_master_rx error code : 0x", err_code);
	PRINT_K("\nReceive Complete\n");
	b_i2ctransferComplete = TRUE;
 }
