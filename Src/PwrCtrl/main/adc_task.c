/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <esp_log.h>


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channel = ADC_CHANNEL_0;     // Utea
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


void adc_task(void)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) 
    {
        uint32_t adc_reading = 0;

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) 
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }

        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        ESP_LOGI("ADC", "Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



void start_adc_task(void)
{
    xTaskCreatePinnedToCore(adc_task, "adc_task", 2048, NULL, 2, NULL, 1);
}

