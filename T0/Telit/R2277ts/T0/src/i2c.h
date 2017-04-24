#ifndef I2C_H
#define I2C_H

#define ACC_I2C_IFACE              LPC_I2C1

#define MMA8652_READ_ADDR          0x3B
#define MMA8652_WRITE_ADDR         0x3A
void sendI2CMaster(uint8_t *sendBuffer, uint16_t sendLen, bool stop);

void readI2CMaster(uint8_t *buffer, uint16_t length, bool stop);
void SendReadI2CMaster(uint8_t i2c_addr, uint8_t reg, uint8_t * i2c_buffer, uint16_t sendLen, uint16_t recvLen);

/* 100KHz I2C bit-rate */
#define I2C_BITRATE             (100000)

#endif /* I2C_H */
