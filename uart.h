/*
 * usart.h
 * Provide a 32 bytes buffer asynchronize working USART driver.
 */ 

#ifndef _USART_H_
#define _USART_H_

#include "global.h"

// Define the USART properties.
#ifndef USART_BUF_SIZE
#define USART_BUF_SIZE      256
#endif
#ifndef USART_BAUD_RATE
#define USART_BAUD_RATE     9600
#endif

// Define the generated result.
#define USART_BAUD_VALUE    ((F_CPU>>4)/USART_BAUD_RATE)
// Define the constants.
#define USART_READ_VALID    0

typedef struct Usart
{
    // Consider the buffer could be accessed by both ISR and function,
    // Use volatile as the mutex exclusion.
    volatile uchar   txBuffer[USART_BUF_SIZE]; // Transfer buffer.
    volatile uchar   rxBuffer[USART_BUF_SIZE]; // Receiver buffer.
    // Status buffer.
    volatile uint8_t status;
    volatile uint8_t txWritePos, txReadPos;
    volatile uint8_t rxWritePos, rxReadPos;
} Usart;

static Usart usart;

#define Usart_init() \
    usart.status = 0x00; \
    usart.txWritePos = 0x00; \
    usart.txReadPos = 0x00; \
    usart.rxWritePos = 0x00; \
    usart.rxReadPos = 0x00; \
    UBRR0H = uint8_t (USART_BAUD_VALUE >> 8); \
    UBRR0L = uint8_t (USART_BAUD_VALUE); \
    UCSR0C |= M_BIT(UCSZ01) | M_BIT(UCSZ00); \
    UCSR0B |= M_BIT(TXEN0) | M_BIT(RXEN0) | M_BIT(RXCIE0);

// Check whether the USART has data
#define Usart_hasData() \
    M_IS_SET(usart.status, USART_READ_VALID)

// Read byte from the buffer, must check has data first.
#define Usart_readByte(ch) \
    (ch) = usart.rxBuffer[usart.rxReadPos++]; \
    if (usart.rxReadPos >= USART_BUF_SIZE - 1) \
    { \
        usart.rxReadPos = 0; \
    } \
    if (usart.rxReadPos == usart.rxWritePos) \
    { \
        M_UNSET_BIT(usart.status, USART_READ_VALID); \
    }

// Write the byte to write buffer.
#define Usart_writeByte(ch) \
    usart.txBuffer[usart.txWritePos++] = (ch); \
    if(usart.txWritePos >= USART_BUF_SIZE - 1) \
    { \
        usart.txWritePos = 0; \
    } \
    M_SET_BIT(UCSR0B, UDRIE0);

void Usart_writeBytes(uchar *str);

#define Usart_writeStr(str) \
    Usart_writeBytes((uchar *)str)

#endif /* _USART_H_ */
