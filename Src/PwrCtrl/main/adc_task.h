
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/adc.h"


// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================

// Coefficients for converting voltage value to physical values
#define K_U_SETI     210
#define K_U_INV      1
#define K_I_AB       1
#define K_I_TE       1
#define K_U_AB       10
#define K_U_TE       1

// ======================================================================
//  TYPE DEFINITIONS
// ======================================================================

// Index names for param_names[] and params[]
typedef enum {
    ADC_CH_U_SETI = 0,
    ADC_CH_U_INV,   
    ADC_CH_I_AB,
    ADC_CH_I_TE,
    ADC_CH_U_AB,
    ADC_CH_U_TE,

    ADC_LAST_PARAM

} ParamChannels_t;

typedef struct {
    uint32_t raw;           // Raw value. Accumulated on data collection
    uint32_t voltage;       // Resulting voltage after calculating middle value

} ParamValue_t;

// Frame of measured parameters
typedef struct {
    uint32_t Useti;     // mV
    uint32_t Uinv;      // mV
    uint32_t Iab;       // mA
    uint32_t Ite;       // mA
    uint32_t Uab;       // mV
    uint32_t Ute;       // mV

} ParamDataFrame_t;


typedef enum {
    ADC_ST_OK,
    ADC_ST_GET_DATA_BUSY,
    ADC_ST_ERROR

} ADC_status_t;

// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

void ADC_start_task(void);
ADC_status_t ADC_get_values(ParamDataFrame_t *requested_adc_data);



#ifdef __cplusplus
}
#endif


