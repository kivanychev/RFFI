#pragma once

// ======================================================================
// CONSTANTS
// ======================================================================

#define FALSE   0
#define TRUE    1

#define ON      1
#define OFF     0 

// Symbols pf the preamble
#define UART_PRE0   'P'
#define UART_PRE1   'R'
#define UART_PRE2   'E'

// ======================================================================
// TYPES
// ======================================================================


// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

#ifdef __cplusplus
extern "C" {
#endif


void UART_start_task(void);

uint8_t UART_get_fault_state(void);
void UART_clear_fault(void);

uint16_t UART_get_battery_state(void);
void UART_set_Iset_level(uint8_t level);
void UART_set_StartAB(uint8_t level);
void UART_set_StartInv(uint8_t level);



#ifdef __cplusplus
}
#endif
