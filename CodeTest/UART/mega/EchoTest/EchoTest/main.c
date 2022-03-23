/*
 * EchoTest.c
 *
 * Created: 11.12.2019 19:19:04
 * Author : ZorG
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define FOSC                16000000            // MCU Clock Speed
#define UART_BR             9600
#define MYUBRR              FOSC/16/UART_BR-1


int main(void)
{
    unsigned char tmp = 0;
    unsigned char *str = "Mega328\r\n";
    int i = 0;
    int len  = strlen(str);
    
    tmp = (1 << PORTD1); 
    DDRD = tmp;
    
    
    UCSR0A = 0x00;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) + (1 << UCSZ00); // 8 bit data frame глава 20.11.4
    
    UBRR0 = MYUBRR;
    
    sei();
    while (1) 
    {
        /* Put data into buffer, sends the data */
        UDR0 = str[i];
        i++;
        if(i >= len)
        {
            i = 0;
        }      

        /* Wait for empty transmit buffer */
        while ( !(UCSR0A & (1<<UDRE0)) )
        ;

    }             
}


ISR(USART_RX_vect)
{
    unsigned char c = UDR0;
    //UDR0 = c;
}


