/*
 * MMA7455.c
 *
 *  Created on: 29 Tem 2015
 *      Author: admin
 */
#include "board.h"
#include "i2c.h"
#include "MMA7455.h"

uint8_t i2c_txBuffer[16];
uint8_t i2c_rxBuffer[16];

int Trio_MMA7455_init(void)
{
  i2c_txBuffer[0] = MMA7455_CTRLADDR;
  i2c_txBuffer[1] = MMA7455_CTRLREG_VAL;
  i2c_write(MMA7455_I2CADDR, i2c_txBuffer, i2c_rxBuffer, 2, 0);

  i2c_txBuffer[0] = MMA7455_CTRLADDR;
  i2c_txBuffer[1] = (MMA7455_I2CADDR << 1);
  i2c_write(MMA7455_I2CADDR, i2c_txBuffer, i2c_rxBuffer, 2, 1);

  if(i2c_rxBuffer[0] ==  MMA7455_CTRLREG_VAL)
	  return 1;
  else
	  return 0;
}

