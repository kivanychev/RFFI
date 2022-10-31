
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
    int32_t voltage;        // Resulting voltage in milli Volts after calculating middle value

} ParamValue_t;

// Frame of measured parameters in milli Volts
typedef struct {
    int32_t Useti;   
    int32_t Uinv;
    int32_t Iab;
    int32_t Ite;
    int32_t Uab;
    int32_t Ute;

} ParamDataFrame_t;


typedef enum {
    ADC_ST_OK,
    ADC_ST_GET_DATA_BUSY,
    ADC_ST_ERROR

} ADC_status_t;


// Coefficients for converting voltage value to physical values
typedef struct {
    uint16_t U_SETI;
    uint16_t U_INV;
    uint16_t I_AB;
    uint16_t I_TE;
    uint16_t U_AB;
    uint16_t U_TE;
    uint16_t STAB_U_AB;

} ADC_coeff_t;

// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

ADC_coeff_t *ADC_init_coeff();
void ADC_update_coeff();

void ADC_start_task(void);
ADC_status_t ADC_get_values(ParamDataFrame_t *requested_adc_data);


#ifdef __cplusplus
}
#endif


