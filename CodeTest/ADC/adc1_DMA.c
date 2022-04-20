/*

Tis code has some problems when measuring  several channels.
For the constant value of the signal connected to the Channel 0 it measures values tha differ more than 50%
This behavior is reproduced on 2 boards in a similar way.

For the case when I specify only 1 channel the measured values are OK.


Uab: Raw = 957  --> Uab: Raw = 1914

.[0;32mI (225710) ADC DMA: --------------------------------------------.[0m
.[0;32mI (226020) ADC DMA: Invalig state.[0m
.[0;32mI (226020) ADC DMA: Uab: Raw = 957, Voltage = 913 mV.[0m
.[0;32mI (226020) ADC DMA: Uinv: Raw = 0, Voltage = 142 mV.[0m
.[0;32mI (226020) ADC DMA: Iab: Raw = 848, Voltage = 825 mV.[0m
.[0;32mI (226030) ADC DMA: Ite: Raw = 84, Voltage = 210 mV.[0m
.[0;32mI (226030) ADC DMA: Useti: Raw = 0, Voltage = 142 mV.[0m
.[0;32mI (226040) ADC DMA: Ute: Raw = 1467, Voltage = 1324 mV.[0m
.[0;32mI (226040) ADC DMA: --------------------------------------------.[0m
.[0;32mI (226350) ADC DMA: Invalig state.[0m
.[0;32mI (226350) ADC DMA: Uab: Raw = 1914, Voltage = 1684 mV.[0m
.[0;32mI (226350) ADC DMA: Uinv: Raw = 0, Voltage = 142 mV.[0m
.[0;32mI (226350) ADC DMA: Iab: Raw = 848, Voltage = 825 mV.[0m
.[0;32mI (226360) ADC DMA: Ite: Raw = 89, Voltage = 214 mV.[0m
.[0;32mI (226360) ADC DMA: Useti: Raw = 0, Voltage = 142 mV.[0m
.[0;32mI (226370) ADC DMA: Ute: Raw = 1470, Voltage = 1326 mV.[0m


*/






/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "adc_task.h"
#include "esp_adc_cal.h"


// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================

#define DEFAULT_VREF        1100

#define TIMES              48
#define GET_UNIT(x)        ((x>>3) & 0x1)

#define ADC_RESULT_BYTE     2
#define ADC_CONV_LIMIT_EN   1                       //For ESP32, this should always be set to 1
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1  //ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1


// ======================================================================
// LOCAL VARIABLES
// ======================================================================

static esp_adc_cal_characteristics_t *adc_chars;

// channel[] Specifies channel numbers for the DMA to collect data from
//static const adc_channel_t channels[6] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7 };
static const adc_channel_t channels[] = {ADC_CHANNEL_0};

static char* param_names[6] = {"Uab", "Uinv", "Iab", "Ite", "Useti", "Ute"};
static ParamValue_t params[6];

static uint16_t adc1_chan_mask = BIT(7);
static uint16_t adc2_chan_mask = 0;
static const char *TAG = "ADC DMA";
static esp_adc_cal_characteristics_t *adc_chars;

// ======================================================================
// FUNCTION PROTOTYPES
// ======================================================================

static void adc_task(void *);

// ======================================================================
// EXTERNAL FUNCTION DEFINITIONS
// ======================================================================

void ADC_get_values(uint16_t *requested_adc_data)
{
    if(requested_adc_data == NULL)
    {
        return;
    }

    // Copy all ADC measured data into external memory
}



/**
 * @brief Starting ADC task
 * 
 */
void ADC_start_task(void)
{
    xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 2, NULL, 0);
}

// ======================================================================
// LOCAL FUNCTION DEFINITIONS
// ======================================================================


/**
 * @brief Clear all parameters data
 * 
 */
static void clear_all_params(void)
{
    for(int paramId = 0; paramId < ADC_LAST_PARAM; paramId++)
    {
        params[paramId].cnt = 0;
        params[paramId].raw = 0;
        params[paramId].voltage = 0;
    }
}


/**
 * @brief ADC initialization: Continous reading with DMA
 * 
 * @param adc1_chan_mask - 
 * @param adc2_chan_mask - 
 * @param channel - IDs array of Channels to read
 * @param channel_num - Number of channels to read
 */
static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = TIMES,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = 250,
        .sample_freq_hz = 50 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;

    for (int i = 0; i < channel_num; i++)
    {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }

    // Calibration values
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}


/**
 * @brief ADC task: Collecting data with DMA
 * 
 */
static void adc_task(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channels, sizeof(channels) / sizeof(adc_channel_t));
    adc_digi_start();

    while(1) {

        ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);

        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) 
        {
            if (ret == ESP_ERR_INVALID_STATE){
                ESP_LOGI(TAG, "Invalig state");
            }

            // ESP_LOGI("TASK:", "ret is %x, ret_num is %d", ret, ret_num);

            clear_all_params();

            for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) 
            {
                adc_digi_output_data_t *p = (void*)&result[i];

                switch(p->type1.channel)
                {
                    case ADC1_CHANNEL_0:
                        params[ADC_CH_U_AB].raw += p->type1.data;
                        params[ADC_CH_U_AB].cnt++;
                        break;

                    case ADC_CHANNEL_3:
                        params[ADC_CH_U_INV].raw += p->type1.data;
                        params[ADC_CH_U_INV].cnt++;
                        break;

                    case ADC_CHANNEL_4:
                        params[ADC_CH_I_AB].raw += p->type1.data;
                        params[ADC_CH_I_AB].cnt++;
                        break;

                    case ADC_CHANNEL_5:
                        params[ADC_CH_I_TE].raw += p->type1.data;
                        params[ADC_CH_I_TE].cnt++;
                        break;

                    case ADC_CHANNEL_6:
                        params[ADC_CH_U_SETI].raw += p->type1.data;
                        params[ADC_CH_U_SETI].cnt++;
                        break;

                    case ADC_CHANNEL_7:
                        params[ADC_CH_U_TE].raw += p->type1.data;
                        params[ADC_CH_U_TE].cnt++;
                        break;

                    default:
                        break;
                }
            }


            for(int paramId = 0; paramId < ADC_LAST_PARAM; paramId++)
            {
                if(params[paramId].cnt == 0)
                {
                    continue;
                }

                // Middle value
                params[paramId].raw /= params[paramId].cnt;

                // Get voltage in milli Volts
                params[paramId].voltage = esp_adc_cal_raw_to_voltage(params[paramId].raw, adc_chars);
                

                ESP_LOGI(TAG, "%s: Raw = %d, Voltage = %d mV", param_names[paramId], params[paramId].raw, params[paramId].voltage);

            }
           ESP_LOGI(TAG, "--------------------------------------------");


            vTaskDelay(30);

        } 
        else if (ret == ESP_ERR_TIMEOUT) 
        {
            /**
             * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
             * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
             */
            ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
            vTaskDelay(1000);
        }

    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
}


