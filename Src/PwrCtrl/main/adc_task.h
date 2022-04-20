
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/adc.h"


// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================



// ======================================================================
//  TYPE DEFINITIONS
// ======================================================================

// Index names for param_names[] and params[]
typedef enum {
    ADC_CH_U_AB = 0,
    ADC_CH_U_INV,   
    ADC_CH_I_AB,
    ADC_CH_I_TE,
    ADC_CH_U_SETI,
    ADC_CH_U_TE,

    ADC_LAST_PARAM

} ParamChannels_t;

typedef struct {
    uint32_t raw;           // Raw value. Accumulated on data collection
    uint32_t voltage;       // Resulting voltage after calculating middle value
    uint32_t cnt;           // Counter of the values

} ParamValue_t;


// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

void ADC_start_task(void);
void ADC_get_values(uint16_t *requested_adc_data);



#ifdef __cplusplus
}
#endif


