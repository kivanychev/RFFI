
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <esp_log.h>

#include "adc_task.h"


// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================


#define DEFAULT_VREF            1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES           4           //Multisampling for other signals
#define NO_OF_DATA_FRAMES       20
#define NO_OF_SAMPLES_ALT       2           //Multisampling for alternating signals
#define NO_OF_DATA_FRAMES_ALT   48           //Number of data frames to read at 1 measure procedure for alternating signals

#define TRUE                    1
#define FALSE                   0

// ======================================================================
// MACROS DEFINITIONS
// ======================================================================

#define DataFrameIsReady()          get_data_request_flag == FALSE
#define DataFrameRequestComplete()  get_data_request_flag = FALSE
#define ThereIsDataFrameRequest()   get_data_request_flag == TRUE

// ======================================================================
// LOCAL VARIABLES
// ======================================================================

ParamDataFrame_t param_data_frame;

// Coefficients for converting voltage value to physical values
static uint32_t k_Useti = 1;
static uint32_t k_Uinv = 1;
static uint32_t k_Iab = 1;
static uint32_t k_Ite = 1;
static uint32_t k_Uab = 1;
static uint32_t k_Ute = 1;

// Used as a request flag for getting measured data by ADC_get_values()
static int get_data_request_flag = FALSE;

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channels[6] = { ADC_CHANNEL_6, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_0, ADC_CHANNEL_7 };

static char* param_names[6] = {"Useti", "Uinv", "Iab", "Ite", "Uab", "Ute"};
static ParamValue_t params[6];

// Collects data for 1 measurement of sine form params:
static uint32_t sine_params_raw[2][NO_OF_DATA_FRAMES_ALT];

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static char *TAG = "ADC";

// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================


// ======================================================================
// LOCAL FUNCTION DEFINITIONS
// ======================================================================

/**
 * @brief Check efuse for ADC calibration
 * 
 */
static void check_efuse(void)
{

    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: Supported\n");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: NOT supported\n");
    }

}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref\n");
    }
}


/**
 * @brief ADC task: Collects measured data
 * 
 * @param args 
 */
void adc_task(void *args)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC channels
    for(int paramId = 0; paramId < ADC_LAST_PARAM; ++paramId)
    {
        adc1_config_width(width);
        adc1_config_channel_atten(channels[paramId], atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) 
    {
        int paramId;

        // Part 1: Measure alternating signals: Useti, Uinv

        // Collecting data frames for some period of time. Useful for sine parameters measurement
        for(int sine_raw_ind = 0; sine_raw_ind < NO_OF_DATA_FRAMES_ALT; sine_raw_ind++)
        {
            for(paramId = 0; paramId < 2; ++paramId)
            {
                params[paramId].raw = 0;

                //Multisampling of data frame for getting average values for each parameter 
                for (int i = 0; i < NO_OF_SAMPLES_ALT; i++) 
                {
                    params[paramId].raw += adc1_get_raw((adc1_channel_t)channels[paramId]);
                }

                // Calculate average values for the raw value
                sine_params_raw[paramId][sine_raw_ind] = params[paramId].raw / NO_OF_SAMPLES_ALT;
            }

        }

        // 1.1 Find maximum value for each sine parameter
        for(paramId = 0; paramId < 2; ++paramId)
        {
            params[paramId].raw = 0;    // Maximum value will be placed here

            for(int sine_raw_ind = 0; sine_raw_ind < NO_OF_DATA_FRAMES_ALT; sine_raw_ind++)
            {
                if(sine_params_raw[paramId][sine_raw_ind] > params[paramId].raw)
                {
                    params[paramId].raw = sine_params_raw[paramId][sine_raw_ind];
                }
            }

            // Get voltage value for Maximum value
            params[paramId].voltage = (esp_adc_cal_raw_to_voltage(params[paramId].raw, adc_chars) * 1000) / 1414;
        }

        // 1.2 Um / sqrt(2)


        // Part 2: Measure other signals

        for(paramId = ADC_CH_I_AB; paramId < ADC_LAST_PARAM; ++paramId)
        {
            params[paramId].raw = 0;

            // Multisampling of data frame for getting average values for each parameter 
            for (int i = 0; i < NO_OF_SAMPLES; i++) 
            {
                params[paramId].raw += adc1_get_raw((adc1_channel_t)channels[paramId]);
            }

            // Calculate average value
            params[paramId].raw /= NO_OF_SAMPLES;
            params[paramId].voltage = esp_adc_cal_raw_to_voltage(params[paramId].raw, adc_chars);
        }

        // Checking if some one wants to get measured data
        if( ThereIsDataFrameRequest() )
        {
            param_data_frame.Uab = params[ADC_CH_U_AB].voltage * k_Uab;
            param_data_frame.Uinv = params[ADC_CH_U_INV].voltage * k_Uinv;
            param_data_frame.Iab = params[ADC_CH_I_AB].voltage * k_Iab;
            param_data_frame.Ite = params[ADC_CH_I_TE].voltage * k_Ite;
            param_data_frame.Useti = params[ADC_CH_U_SETI].voltage * k_Useti;
            param_data_frame.Ute = params[ADC_CH_U_TE].voltage * k_Ute;

            // Reset flag
            DataFrameRequestComplete();
        }

        // Print all parameters
        for(paramId = 0; paramId < ADC_LAST_PARAM; ++paramId)
        {
            //Convert adc_reading to voltage in mV
            ESP_LOGI(TAG, "%s = Raw: %d\tVoltage: %dmV", param_names[paramId], params[paramId].raw, params[paramId].voltage);
        }

        ESP_LOGI(TAG, "---------------------------------");

        vTaskDelay(80);
    }
}

// ======================================================================
// EXTERNAL FUNCTION DEFINITIONS
// ======================================================================

/**
 * @brief Starts ADC task on Core 0
 * 
 */
void ADC_start_task(void)
{
    xTaskCreatePinnedToCore(adc_task, "adc_task", 2048, NULL, 2, NULL, 0);
}

/**
 * @brief 
 * 
 * @param requested_adc_data 
 */
ADC_status_t ADC_get_values(ParamDataFrame_t *requested_adc_data)
{
    static int get_data_first_call_flag = TRUE;

    // Start procedure for getting Parameters data frame
    if(get_data_first_call_flag == TRUE)
    {
        get_data_request_flag = TRUE;

        get_data_first_call_flag = FALSE;   // Next calls will be waiting for the 'get_data_request_flag' to be in reset state
        return ADC_ST_GET_DATA_BUSY;
    }

    if( DataFrameIsReady() )
    {
        get_data_first_call_flag = TRUE;

        if(requested_adc_data == NULL)
        {
            return ADC_ST_ERROR;
        }

        *requested_adc_data = param_data_frame;
    } 

    return ADC_ST_OK;
}