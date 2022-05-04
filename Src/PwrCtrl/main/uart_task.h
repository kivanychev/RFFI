#pragma once

// ======================================================================
// CONSTANTS
// ======================================================================

#define FALSE   0
#define TRUE    1

#define ON      1
#define OFF     0 

// ======================================================================
// TYPES
// ======================================================================

// Parameter codes for sending to ATmega
typedef enum {
    PARAM_CLEAR_FAULT,
    PARAM_START_INV_ON,
    PARAM_START_INV_OFF,
    PARAM_START_AB_ON,
    PARAM_START_AB_OFF,
    PARAM_I_SET

} UART_Param_t;

// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

#ifdef __cplusplus
extern "C" {
#endif


void UART_start_task(void);

uint16_t UART_get_fault_state(void);
void UART_clear_fault(void);

uint16_t UART_get_battery_state(void);
void UART_set_Iset_level(uint32_t level);
void UART_set_StartAB(uint8_t level);
void UART_set_StartInv(uint8_t level);



#ifdef __cplusplus
}
#endif
