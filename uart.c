#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

ISR(USART_RX_vect)
{
    // Write the data to buffer.
    usart.rxBuffer[usart.rxWritePos++] = UDR0;
    // Update the flag.
    M_SET_BIT(usart.status, USART_READ_VALID);
    // Check the position.
    if(usart.rxWritePos >= USART_BUF_SIZE - 1)
    {
        // Reset the position back to the first byte.
        usart.rxWritePos = 0;
    }
}

ISR(USART_UDRE_vect)
{
    // Check whether there is no data need to output.
    if(usart.txReadPos == usart.txWritePos)
    {
        // Disable the interrupt.
        M_UNSET_BIT(UCSR0B, UDRIE0);
    }
    else
    {
        // Write one byte to UDR0.
        UDR0 = usart.txBuffer[usart.txReadPos++];
        if(usart.txReadPos >= USART_BUF_SIZE - 1)
        {
            // Reset the pos.
            usart.txReadPos = 0;
        }
    }
}

void Usart_writeBytes(uchar *str)
{
    // Check the str end position.
    uchar *endPos = str + strlen((const char *)str);
    while(str < endPos)
    {
        // Write the data to the buffer.
        Usart_writeByte(*(str++));
    }
}
