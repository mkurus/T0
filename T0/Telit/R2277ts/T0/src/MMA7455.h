/*
 * MMA7455.h
 *
 *  Created on: 29 Tem 2015
 *      Author: admin
 */

#ifndef MMA7455_H_
#define MMA7455_H_

/* The 7-bit address */
#define MMA7455_I2CADDR 0x1D
/* The mode control register address */
#define MMA7455_CTRLADDR 0x16
/* The control value for 2G Measurement mode */
#define MMA7455_CTRLREG_VAL  0x05

/* The registers to read */
#define MMA7455_XOUT8 0x6
#define MMA7455_YOUT8 0x7
#define MMA7455_ZOUT8 0x8

int Trio_MMA7455_init(void);
#endif /* MMA7455_H_ */
