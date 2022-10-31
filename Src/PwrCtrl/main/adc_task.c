
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "nvs_flash.h"
#include "nvs.h"

#include <esp_log.h>

#include "adc_task.h"


// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================


#define DEFAULT_VREF            1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES           4           //Multisampling for other signals
#define NO_OF_DATA_FRAMES       20
#define NO_OF_SAMPLES_ALT       4           //Multisampling for alternating signals
#define NO_OF_DATA_FRAMES_ALT   48           //Number of data frames to read at 1 measure procedure for alternating signals

#define TRUE                    1
#define FALSE                   0

// Zero offset values in milli Volts
#define I_TE_ZERO_OFFSET        1679
#define I_AB_ZERO_OFFSET        1674

// Coefficient delimiter
#define COEFF_DELIMITER_100     100
#define COEFF_DELIMITER_10      10
#define COEFF_DELIMITER_1       1

// ======================================================================
// MACROS DEFINITIONS
// ======================================================================

#define DataFrameIsReady()          get_data_request_flag == FALSE
#define DataFrameRequestComplete()  get_data_request_flag = FALSE
#define ThereIsDataFrameRequest()   get_data_request_flag == TRUE

// ======================================================================
// LOCAL VARIABLES
// ======================================================================

ADC_coeff_t coeff = {   .I_AB =1, 
                        .I_TE = 1,
                        .U_AB = 1,
                        .U_INV = 1,
                        .U_SETI = 1,
                        .U_TE = 1};


ParamDataFrame_t param_data_frame;

// Used as a request flag for getting measured data by ADC_get_values()
static volatile int get_data_request_flag = FALSE;

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

            // Get voltage value for Maximum immediate value  
            params[paramId].voltage = (int32_t)esp_adc_cal_raw_to_voltage(params[paramId].raw, adc_chars);
        }

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
            params[paramId].voltage = (int32_t)esp_adc_cal_raw_to_voltage(params[paramId].raw, adc_chars);
        }

        // Checking if some one wants to get measured data
        if( ThereIsDataFrameRequest() )
        {
            param_data_frame.Uab = (params[ADC_CH_U_AB].voltage * coeff.U_AB)                       / COEFF_DELIMITER_10;
            param_data_frame.Uinv = (params[ADC_CH_U_INV].voltage * coeff.U_INV)                    / COEFF_DELIMITER_10;
            param_data_frame.Iab = ((params[ADC_CH_I_AB].voltage - I_AB_ZERO_OFFSET) * (int32_t)coeff.I_AB)  / COEFF_DELIMITER_10;
            param_data_frame.Ite = ((params[ADC_CH_I_TE].voltage - I_TE_ZERO_OFFSET) * (int32_t)coeff.I_TE)  / COEFF_DELIMITER_10;
            param_data_frame.Useti = (params[ADC_CH_U_SETI].voltage * coeff.U_SETI)                 / COEFF_DELIMITER_10;
            param_data_frame.Ute = (params[ADC_CH_U_TE].voltage * coeff.U_TE)                       / COEFF_DELIMITER_10;

            // Reset flag
            DataFrameRequestComplete();
            ESP_LOGD(TAG, "Finished data request");
        }

        // Print all parameters
        for(paramId = 0; paramId < ADC_LAST_PARAM; ++paramId)
        {
            //Convert adc_reading to voltage in mV
            ESP_LOGD(TAG, "%s = Raw: %d\tVoltage: %dmV", param_names[paramId], params[paramId].raw, params[paramId].voltage);
        }

        // Print sine raw values
        for(paramId = 0; paramId < 2; ++paramId)
        {
            ESP_LOGD(TAG, "%s raw vaues:", param_names[paramId]);
            for(int sine_raw_ind = 0; sine_raw_ind < NO_OF_DATA_FRAMES_ALT; sine_raw_ind++)
            {
                ESP_LOGD(TAG, "%d", sine_params_raw[paramId][sine_raw_ind]);
            }
        }

        ESP_LOGD(TAG, "---------------------------------");

        vTaskDelay(10);
    }
}

// ======================================================================
// EXTERNAL FUNCTION DEFINITIONS
// ======================================================================

ADC_coeff_t *ADC_init_coeff()
{
    // Read coefficients from NVS

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK( err );

    // Open NVS
    ESP_LOGI(TAG, "Error status:%d\n", err);
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");

    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } 
    else {
        if( nvs_get_u16(nvs_handle, "coeff.U_SETI", &coeff.U_SETI) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.U_SETI not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.U_SETI: read value = %d", coeff.U_SETI);
        }


        if( nvs_get_u16(nvs_handle, "coeff.U_INV", &coeff.U_INV) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.U_INV not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.U_INV: read value = %d", coeff.U_INV);
        }


        if( nvs_get_u16(nvs_handle, "coeff.I_AB", &coeff.I_AB) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.I_AB not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.I_AB: read value = %d", coeff.I_AB);
        }


        if( nvs_get_u16(nvs_handle, "coeff.I_TE", &coeff.I_TE) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.I_TE not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.I_TE: read value = %d", coeff.I_TE);
        }


        if( nvs_get_u16(nvs_handle, "coeff.U_AB", &coeff.U_AB) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.U_AB not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.U_AB: read value = %d", coeff.U_AB);
        }


        if( nvs_get_u16(nvs_handle, "coeff.U_TE", &coeff.U_TE) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.U_TE not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.U_TE: read value = %d", coeff.U_TE);
        }


        if( nvs_get_u16(nvs_handle, "coeff.STAB_U_AB", &coeff.STAB_U_AB) != ESP_OK) {
            ESP_LOGI(TAG, "coeff.U_TE not yet found");
        }
        else {
            ESP_LOGI(TAG, "coeff.STAB_U_AB: read value = %d", coeff.STAB_U_AB);
        }

        nvs_close(nvs_handle);
    }

    return &coeff;
}

/**
 * @brief Writes new coefficients data to NVS
 * 
 */
void ADC_update_coeff()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK( err );

    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } 
    else 
    {
        err = nvs_set_u16(nvs_handle, "coeff.U_SETI", coeff.U_SETI);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "U_SETI Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "U_SETI Write Successful!");
        }

        err = nvs_set_u16(nvs_handle, "coeff.U_INV", coeff.U_INV);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "U_INV Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "U_INV Write Successful!");
        }

        err = nvs_set_u16(nvs_handle, "coeff.I_AB", coeff.I_AB);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "I_AB Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "I_AB Write Successful!");
        }

        err = nvs_set_u16(nvs_handle, "coeff.I_TE", coeff.I_TE);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "I_TE Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "I_TE Write Successful!");
        }

        err = nvs_set_u16(nvs_handle, "coeff.U_AB", coeff.U_AB);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "U_AB Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "U_AB Write Successful!");
        }

        err = nvs_set_u16(nvs_handle, "coeff.U_TE", coeff.U_TE);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "U_TE Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "U_TE Write Successful!");
        }

        err = nvs_set_u16(nvs_handle, "coeff.STAB_U_AB", coeff.STAB_U_AB);
        if(err != ESP_OK) {
            ESP_LOGD(TAG, "STAB_U_AB Write Failed!");
        }
        else {
            ESP_LOGD(TAG, "STAB_U_AB Write Successful!");
        }


        err = nvs_commit(nvs_handle);
        if(err != ESP_OK)
        {
            ESP_LOGD(TAG, "Commit Failed!");
        }
        else
        {
            ESP_LOGD(TAG, "Commit Successful!");
        }

        // Close
        nvs_close(nvs_handle);
    }

}

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