
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <esp_log.h>


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   2           //Multisampling

// Index values for channels[] and param_values[] array
typedef enum {
    ADC_CH_U_AB = 0,
    ADC_CH_U_INV,   
    ADC_CHA_I_AB,
    ADC_CHA_I_TE,
    ADC_CHA_U_SETI,
    ADC_CHA_U_TE,

    ADC_LAST_PARAM

} ParamChannels_t;


static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channels[6] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7 };
static char* param_names[6] = {"Uab", "Uinv", "Iab", "Ite", "Useti", "Ute"};
static uint16_t param_values[6];

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static void check_efuse(void)
{

    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI("ADC", "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI("ADC", "eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI("ADC", "eFuse Vref: Supported\n");
    } else {
        ESP_LOGI("ADC", "eFuse Vref: NOT supported\n");
    }

}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI("ADC", "Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI("ADC", "Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI("ADC", "Characterized using Default Vref\n");
    }
}


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
        for(int contunous_read_cnt = 0; contunous_read_cnt < 500; contunous_read_cnt++)
        {
            for(int paramId = 0; paramId < ADC_LAST_PARAM; ++paramId)
            {
                uint32_t adc_reading = 0;

                //Multisampling
                for (int i = 0; i < NO_OF_SAMPLES; i++) 
                {
                    adc_reading += adc1_get_raw((adc1_channel_t)channels[paramId]);
                }

                param_values[paramId] = adc_reading / NO_OF_SAMPLES;
            }

        }

        // Print all parameters
        for(int paramId = 0; paramId < ADC_LAST_PARAM; ++paramId)
        {
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(param_values[paramId], adc_chars);
            ESP_LOGI("ADC", "%s = Raw: %d\tVoltage: %dmV", param_names[paramId], param_values[paramId], voltage);
            

        }

        ESP_LOGI("ADC", "---------------------------------");

        vTaskDelay(1);
    }
}



void ADC_start_task(void)
{
    xTaskCreatePinnedToCore(adc_task, "adc_task", 2048, NULL, 2, NULL, 0);
}

