/*
 * utils.h
 *
 *  Created on: 30 Tem 2015
 *      Author: admin
 */

#ifndef UTILS_H_
#define UTILS_H_

/* function prototypes */
void Init_DEBUG_USART();
void PRINT_K(const char *);
void PRINT_LN(char *msg);
void PRINT_INT(uint32_t);
void Trio_DebugPortTask();
void Deactivate_GPS_LED();
void Activate_GPS_LED();
void DumpStackPointer();
uint16_t Get_UartLine(LPC_USART_T *uart, RINGBUFF_T *ringBuffer, char *outBuffer,uint16_t buffSize, char startChar, char endChar, uint16_t timeout);

void Delay(TIMER_TICK_T delay, void (*callbackFunc)());

void Board_LED_Init(void);

void Board_LED_Toggle(uint8_t LEDNumber);

int  Hex2Str(char *dest, uint32_t val);
unsigned long Str2Hex(unsigned char *str);
unsigned char Str2HexNum(unsigned char c);

void Print_Val(const char *str, uint32_t val);
uint32_t calc_chksum(uint8_t *start_addr, uint16_t length);
void SetBit(uint32_t *x, uint32_t bitNum);
void ResetBit(uint32_t *x, uint32_t bitNum);
uint16_t crc16(uint8_t *p_data, uint16_t length);

/* extract the low or high byte out of a word16 */
#define HIGHBYTE(w)     ((uint8_t)((w)>>8))
#define LOWBYTE(w)      ((uint8_t)((w)&0x00ff))

/* extract the low or high word16 out of a word32 */
#define HIGHWORD(l)     ((uint16_t)(((uint32_t)(l) >> 16) & 0x0000FFFF))
#define LOWWORD(l)      ((uint8_t)((l) & 0x0000FFFF))

/* extract bytes out of a word32 */
#define WORD32_BYTE0(l) ((uint8_t)(((uint32_t)(l)) & 0x000000FF))
#define WORD32_BYTE1(l) ((uint8_t)(((uint32_t)(l) >>  8) & 0x000000FF))
#define WORD32_BYTE2(l) ((uint8_t)(((uint32_t)(l) >> 16) & 0x000000FF))
#define WORD32_BYTE3(l) ((uint8_t)(((uint32_t)(l) >> 24) & 0x000000FF))

/* convert a byte to a word32 and shift it left one byte */
#define WORD32_LSHIFT8(B)   ( (uint32_t)((uint32_t)(B)<<8) )

/* convert a byte to a word32 and shift it left two bytes */
#define WORD32_LSHIFT16(B)  ( (uint32_t)((uint32_t)(B)<<16) )

/* convert a byte to a word32 and shift it left three bytes */
#define WORD32_LSHIFT24(B)  ( (uint32_t)((uint32_t)(B)<<24) )

/* convert 4 bytes to a word32 */
#define MK_WORD32(A,B,C,D)  ((uint32_t)( ((uint32_t)(A)<<24) | ((uint32_t)(B)<<16) | ((uint32_t)(C)<<8) | (D) ))
#define MK_WORD16(A,B)      ((uint16_t)( ((uint16_t)(A)<<8) | (B) ))

#define GPS_LED_PIN              8

#define TRIO_CONFIG_WORD         "#SET"
#define TRIO_GET_COMMAND         "#GET;INFO"

#endif /* UTILS_H_ */
