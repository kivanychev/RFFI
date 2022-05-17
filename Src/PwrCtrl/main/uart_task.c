/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "uart_task.h"
#include "gpio_control.h"

#include <esp_log.h>

// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================

#define ECHO_TEST_TXD  (GPIO_NUM_17)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (128)

// UART message data length
#define DATA_LEN_IN             2
#define DATA_LEN_OUT            2

// ======================================================================
//  TYPE DEFINITIONS
// ======================================================================

// Codes for parameters coming from Main module
typedef enum {
    PARAM_ClearFault,
    PARAM_StartInv_ON,
    PARAM_StartInv_OFF,
    PARAM_StartAB_ON,
    PARAM_StartAB_OFF,
    PARAM_Iset
} PARAM_code_t;

typedef enum {
    UART_GET_PREAMBLE,
    UART_GET_DATA
} UART_state_t;

// ======================================================================
// LOCAL VARIABLES
// ======================================================================



// ======================================================================
// FUNCTION DEFINITIONS
// ======================================================================
/**
 * @brief 
 * 
 * @param 
 * @return 
 */

/**
 * @brief Receives data from Peripheral controller
 * 
 * @param arg
 */
static void uart_task(void *arg)
{
    /* Configure parameters of UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    // Configure a temporary buffer for the incoming data
    uint8_t *uart_buf = (uint8_t *) malloc(BUF_SIZE);

    // UART enable bit
    UARTenable();

    ESP_LOGD("UART", "Started");

    uint8_t data[DATA_LEN_IN];
    uint8_t ind_pre;
    uint8_t ind_data;

    UART_state_t uart_state = UART_GET_PREAMBLE;

    unsigned char preamble[] = { 0xFF, 0xFF };      //Preamble for sent/received data over UART


    while (1) 
    {
        int len = uart_read_bytes(UART_NUM_1, &(uart_buf[0]), BUF_SIZE, 20 / portTICK_RATE_MS);

        ind_pre = 0;
        ind_data = 0;

        if(len > 0)
        {
            for(int ind = 0; ind < len; ind++)
            {
                if(uart_state == UART_GET_PREAMBLE)
                {
                    if(uart_buf[ind] == preamble[ind_pre])
                    {
                        ind_pre++;

                        if(ind_pre >= sizeof(preamble))
                        {
                            uart_state = UART_GET_DATA;
                            ind_pre = 0;
                        }
                    }
                    else    // Not a preamble
                    {
                        ind_pre = 0;
                    }
                }
                else if(uart_state == UART_GET_DATA)
                {
                    data[ind_data] = uart_buf[ind];
                    ind_data++;

                    if(ind_data >= DATA_LEN_IN)
                    {
                        ind_data = 0;
                        uart_state = UART_GET_PREAMBLE;
                    }
                }
            }

            ESP_LOGI("UART", "%2X %2X, len = %d", data[0], data[1], len);  
        }

        vTaskDelay(100);

        // Write data back to the UART
        //uart_write_bytes(UART_NUM_1, (const char *) data, len);
    }
}


/**
 * @brief Starts UART task
 */
void UART_start_task(void)
{
    xTaskCreatePinnedToCore(uart_task, "uart_task", 2048, NULL, 10, NULL, 0);
}

/**
 * @brief 
 * 
 * @param 
 * @return 
 */
uint16_t UART_get_fault_state(void)
{
    uint8_t fault_state = FALSE;

    return fault_state;
}


/**
 * @brief 
 * 
 * @param 
 * @return 
 */
void UART_clear_fault(void)
{

}


/**
 * @brief 
 * 
 * @param 
 * @return 
 */
uint16_t UART_get_battery_state(void)
{
    uint16_t battery_state = 0;

    return battery_state;
}


/**
 * @brief 
 * 
 * @param 
 * @return 
 */
void UART_set_Iset_level(uint32_t level)
{

}


/**
 * @brief 
 * 
 * @param 
 * @return 
 */
void UART_set_StartAB(uint8_t level)
{

}


/**
 * @brief 
 * 
 * @param 
 * @return 
 */
void UART_set_StartInv(uint8_t level)
{

}


