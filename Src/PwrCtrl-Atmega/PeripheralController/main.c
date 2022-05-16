/*
 * PeripheralController.c
 *
 * Created: 16.05.2022 11:04:27
 * Author : Kirill Ivanychev
 * (c) NNTU, 2022
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

//---------------------------------------------------------------------------------------
// CONSTANT DEFINITIONS
//---------------------------------------------------------------------------------------


#define FOSC                16000000            // MCU Clock Speed
#define UART_BR             19200
#define UART0_UBRR          FOSC/16/UART_BR-1


// Pins on PortB
#define B_AT_BAT9_PIN           0
#define B_START_AB              1
#define B_START_INV             2
#define B_CLR_FAULT             4
#define B_FAULT                 5


// Pins on PortC
#define C_AT_BAT4_PIN           5
#define C_AT_BAT6_PIN           4
#define C_AT_BAT8_PIN           3
#define C_AT_BAT10_PIN          2
#define C_AT_BAT11_PIN          1
#define C_AT_BAT12_PIN          0


// Pins on PortD
#define D_TXD0                  1
#define D_AT_BAT1_PIN           3
#define D_AT_BAT2_PIN           2
#define D_AT_BAT3_PIN           4
#define D_AT_BAT5_PIN           5
#define D_AT_BAT7_PIN           7
#define D_I_SET_PIN             6

// UART message data length
#define DATA_LEN_IN             2
#define DATA_LEN_OUT            2

// Index pf the parameter code in the received message
#define PARAM_CODE_IND          0
#define PARAM_VALUE_IND         1

#define I_SET_INITIAL_LEVEL     0x3F

#define START                   1
#define STOP                    0

//---------------------------------------------------------------------------------------
//      MACROS
//---------------------------------------------------------------------------------------

// Fault ON -> 0, Fault Off -> 1
#define ClearFault_ON()     PORTB &= (0xFF - (1 << B_FAULT) )
#define ClearFault_OFF()    PORTB |= (1 << B_FAULT)

#define StartInv_ON()       PORTB |= (1 << B_START_INV)
#define StartInv_OFF()      PORTB &= (0xFF - (1 << B_START_INV) )

#define StartAB_ON()        PORTB |= (1 << B_START_AB)
#define StartAB_OFF()       PORTB &= (0xFF - (1 << B_START_AB) )

#define I_set(level)        OCR0A = level

//---------------------------------------------------------------------------------------
//      TYPE DEFINITIONS
//---------------------------------------------------------------------------------------

typedef enum {
    UART_GET_PREAMBLE,
    UART_GET_DATA
} UART_state_t;

// Codes for parameters coming from Main module
typedef enum {
    PARAM_ClearFault,
    PARAM_StartInv_ON,
    PARAM_StartInv_OFF,
    PARAM_StartAB_ON,
    PARAM_StartAB_OFF,
    PARAM_Iset
} PARAM_code_t;



//---------------------------------------------------------------------------------------
//      LOCAL VARIABLES
//---------------------------------------------------------------------------------------

//Preamble for sent/received data over UART
unsigned char preamble[] = { 0xFF, 0xFF };
unsigned char data_buffer_in[DATA_LEN_IN];
unsigned char data_buffer_out[DATA_LEN_OUT];

// Flag is set on ClearFault command reception. It is intended for controlling ClearFault signal via Timer1
unsigned char f_clear_fault = STOP;



//---------------------------------------------------------------------------------------
//      FUNCTION PROTOTYPES
//---------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------
//      FUNCTION DEFINITIONS
//---------------------------------------------------------------------------------------
/**
 * @brief
 * 
 * @param 
 * @param
 * @return
 */

/**
 * @brief Initializes timer 0 for Fast PWM mode, 20 kHz
 * 
 */
void Timer0_Init(void)
{
    // Set PWM frequency to 62.5 kHz at OC0A
    TCCR0A = (1 << COM0A1) + (1 << WGM01) + (1 << WGM00);
    
    // Clock is set to 16 MHz
    TCCR0B = (1 << CS00);
 
    OCR0A = I_SET_INITIAL_LEVEL;    
}

/**
 * @brief Initializes timer 1 for handling time features
 *        Interrupts every 10ms
 */
void Timer1_Init(void)
{
    /********************************/
    /* Initialize Timer1            */
    /* Measures time                */
    /* Clock prescaler CKL/256      */
    /* Operation mode: CTC: WGM=0100*/
    /* Interrupt on Compare Match   */
    /********************************/

    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);    //CTC mode CLK / 256
    TIMSK1 = (1 << OCIE1A);                 // Разрешить вызов прерывания
    OCR1A = 625;                            // 625 * 16 us = 10 000 us = 10 ms (tick time)
}


/**
 * @brief Initializes UART0 for sending and receiving commands from Main module
 * 
 */
void UART0_Init(void)
{
    // Data frame: 8 Data, 1 Stop, No Parity
    UCSR0A = 0x00;

    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) + (1 << UCSZ00);                 // 8 bit data frame глава 20.11.4

    UBRR0H = UART0_UBRR >> 8;
    UBRR0L = UART0_UBRR;
}

/**
 * @brief Sends string over UART0
 * 
 * @param str - String for sending to UART0
 */
void UART0_SendStr(char *str)
{
    static unsigned char ind;

    //cli();
    for(ind = 0; str[ind] != '\0'; ++ind)
    {
        // wait for data to be received
        while( !(UCSR0A & (1<<UDRE0)) );

        // send data
        UDR0 = str[ind];
    }
    //sei();
}




/**
 * @brief Initializes GPIOs for the controller
 * 
 */
void GPIO_Init(void)
{
    // Setting output pins for PortB
    DDRB = (1 << B_START_AB) | (1 << B_START_INV) | (1 << D_TXD0) | (1 << B_CLR_FAULT);

    // Setting output pins for PortD
    DDRD = (1 << D_I_SET_PIN);

}

/**
 * @brief Starts Clear Fault pulse generation
 * 
 */
void ClearFault(void)
{

}


//---------------------------------------------------------------------------------------
// INTERRUPT HANDLERS
//-----------------------------------------------------------------------------------------
/**
 * @brief  Receives commands from Main controller
 * 
 */
ISR(USART_RX_vect)
{
    static UART_state_t uart_state = UART_GET_PREAMBLE;

    short ind = 0;

    // Read byte from UART
    unsigned char c = UDR0;

    if(uart_state == UART_GET_PREAMBLE)
    {
        if( c == preamble[ind])
        {
            ind++;
            if(ind >= sizeof(preamble))
            {
                ind = 0;
                uart_state = UART_GET_DATA;            
            }
        }
        else // Not a preamble: Search for preamble again
        {
            ind = 0;
        }

    }
    else if(uart_state == UART_GET_DATA)
    {
        data_buffer_in[ind] = c;
        ind++;

        PARAM_code_t param_code = (PARAM_code_t)(data_buffer_in[PARAM_CODE_IND]);
        unsigned char param_value = data_buffer_in[PARAM_VALUE_IND];

        if(ind >= DATA_LEN_IN)
        {
            // Apply received parameter
            switch(param_code)
            {
                case PARAM_ClearFault:
                    // Start ClearFault pulse generation via Timer1 interrupt handler
                    f_clear_fault = START;
                    break;

                case PARAM_StartInv_ON:
                    StartInv_ON();
                    break;

                case PARAM_StartInv_OFF:
                    StartInv_OFF();
                    break;

                case PARAM_StartAB_ON:
                    StartAB_ON();
                    break;

                case PARAM_StartAB_OFF:
                    StartAB_OFF();
                    break;

                case PARAM_Iset:
                    I_set(param_value);
                    break;

                default:
                    break;
            }

            // Next time Search for the next preamble 
            ind = 0;
            uart_state = UART_GET_PREAMBLE;
        }
    }

}




/**
 * @brief Handles time features
 * 
 */
ISR(TIMER1_COMPA_vect)
{
    ClearFault_OFF();

    if(f_clear_fault == START)
    {
        ClearFault_ON();
    }
}


//---------------------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------------------


int main(void)
{
    GPIO_Init();
    UART0_Init();
    Timer0_Init();
    
    // Initialize Clear Fault signal to 1 (OFF)
    ClearFault_OFF();

    sei();

    while (1) 
    {
        UART0_SendStr("Mega\r\n");
    }             
}

