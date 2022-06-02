/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "sine_timer.h"

#include <esp_log.h>

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

#define LED_GPIO    13

// UART level converter enable
#define GPIO_OUTPUT_UART_EN    21
#define GPIO_OUTPUT_PIN_SEL  ((1ULL << LED_GPIO) | (1ULL << GPIO_OUTPUT_UART_EN))

#define BOOT_PIN_GPIO           0
#define SB2_GPIO                15
#define SB1_GPIO                2
#define GPIO_INPUT_PIN_SEL      ((1ULL << BOOT_PIN_GPIO) | (1ULL << SB2_GPIO) )
#define ESP_INTR_FLAG_DEFAULT   0


static xQueueHandle gpio_evt_queue = NULL;


/**
 * @brief Handles interrupt for GPIO
 * 
 * @param arg - Not used
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief Receives messages for changing GPIO state
 * 
 * @param arg - Not used
 */
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    float scale = 0.9 * MAX_SINE_AMPLITUDE;
    float step = -2.0;

    while(1) 
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
        {
            if(io_num == SB2_GPIO)
            {
                if(scale >= (MAX_SINE_AMPLITUDE - MIN_SINE_AMPLITUDE) )
                {
                    step = -2.0;
                }

                if(scale <= MIN_SINE_AMPLITUDE)
                {
                    step = 2;
                }

                scale += step;
                ESP_LOGI("GPIO", "\r\nSet amplitude scale to %f", scale);
                Sine_set_amplitude(scale);

            }
        }

    vTaskDelay(5);
    }
}

/**
 * @brief Sets level of the LED
 * 
 * @param state - state to be set for LED
 */
void LEDstate(uint32_t state)
{
    gpio_set_level(LED_GPIO, state);
}

/**
 * @brief Enables UART level converter coming from ATmega328P
 * 
 */
void UARTenable(void)
{
    gpio_set_level(GPIO_OUTPUT_UART_EN, 1);
}


/**
 * @brief Main GPIO test function: Configures GPIO direction, interrupts
 * 
 */
void GPIO_init(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO13
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO2/15 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(BOOT_PIN_GPIO, GPIO_INTR_NEGEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(SB2_GPIO, gpio_isr_handler, (void*) SB2_GPIO);

}
