/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_http_server.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <stdio.h>

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "driver/gpio.h"

#include "wifi_ap.h"
#include "gpio_control.h"
#include "pwm_control.h"
#include "sine_timer.h"
#include "adc_task.h"
#include "uart_task.h"
#include "enc28j60_task.h"

#define TRUE    1
#define FALSE   0
#define BAT_OK      '+'
#define BAT_FAILURE '-'

// ===================================================================
// LOCAL VARIABLES
// ===================================================================

static const char *TAG = "MAIN";
volatile int http_response_active = FALSE;

ParamDataFrame_t system_params;

// 
volatile uint8_t startInv = FALSE;
volatile uint8_t startAB = FALSE;
volatile uint8_t batteries[12] = {BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK, 
                                  BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK};

// ===================================================================
// LOCAL FUNCTIONS
// ===================================================================

/**
 * @brief Parses GET request result
 * 
 * @param req       request instance pointer
 * @param obuf      output buffer for placing parse result
 * @return esp_err_t 
 */
static esp_err_t parse_get(httpd_req_t *req, char **obuf)
{
    char *buf = NULL;
    size_t buf_len = 0;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            *obuf = buf;
            return ESP_OK;
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}


// -----------------------------------------------
// STATUS HANDLER
// -----------------------------------------------


/* Status GET handler */
static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    uint16_t bat_state = UART_get_battery_state();
    uint8_t flt_state = UART_get_fault_state();

    // Reading batteries state into string form
    for(int ind_b = 0; ind_b < 12; ++ind_b)
    {
        if( (bat_state & (1 << ind_b)) == 0) 
        {
            batteries[ind_b] = BAT_OK;
        }
        else
        {
            batteries[ind_b] = BAT_FAILURE;
        }
    }

    char *p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"u_seti\":%u.%u,", system_params.Useti/1000, system_params.Useti % 1000);
    p += sprintf(p, "\"u_inv\":%u.%u,", system_params.Uinv/1000, system_params.Uinv % 1000);
    p += sprintf(p, "\"i_te\":%u.%u,", system_params.Ite/1000, system_params.Ite % 1000);
    p += sprintf(p, "\"u_te\":%u.%u,", system_params.Ute/1000, system_params.Ute % 1000);
    p += sprintf(p, "\"u_ab\":%u.%u,", system_params.Uab/1000, system_params.Uab % 1000);
    p += sprintf(p, "\"i_ab\":%u.%u,", system_params.Iab/1000, system_params.Iab % 1000);   
    p += sprintf(p, "\"batteries\":\"%c%c%c%c %c%c%c%c %c%c%c%c\",", batteries[0], batteries[1], batteries[2],
                                                                     batteries[3], batteries[4], batteries[5],
                                                                     batteries[6], batteries[7], batteries[8],
                                                                     batteries[9], batteries[10], batteries[11]);
    p += sprintf(p, "\"invertor-state-text\":%s,", UART_get_fault_state() == 1 ? "\"Норма\"" : "\"Ошибка\"" );
    p += sprintf(p, "\"eth-ip-text\":\"%s\",", ENC28J60_getEthernetIP());
    p += sprintf(p, "\"start_inv\":%u,", startInv);
    p += sprintf(p, "\"start_ab\":%u", startAB);

    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}


static const httpd_uri_t status = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_handler,
    .user_ctx  = NULL
};

// -----------------------------------------------
// CLEAR FAULT HANDLER
// -----------------------------------------------

static esp_err_t clear_fault_handler(httpd_req_t *req)
{
    UART_clear_fault();

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static const httpd_uri_t clear_fault_uri = {
    .uri       = "/clear-fault",
    .method    = HTTP_GET,
    .handler   = clear_fault_handler,
    .user_ctx  = NULL
};



// -----------------------------------------------
// PAGE HANDLER
// -----------------------------------------------


/* Page from file index.html
 */
static esp_err_t index_handler(httpd_req_t *req)
{
    extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
    extern const unsigned char index_html_gz_end[] asm("_binary_index_html_gz_end");
    size_t index_html_gz_len = index_html_gz_end - index_html_gz_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_len);
}

static const httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
};


// -----------------------------------------------
// SET SINE SCALE HANDLER
// -----------------------------------------------


static esp_err_t sine_scale_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _sine_scale[32];

    if (parse_get(req, &buf) != ESP_OK ||
            httpd_query_key_value(buf, "sine_scale", _sine_scale, sizeof(_sine_scale)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int sine_scale = atoi(_sine_scale);

    if(sine_scale > 95)
    {
        sine_scale = 95;
    }

    if(sine_scale < 5)
    {
        sine_scale = 5;
    }

    ESP_LOGI(TAG, "Set Sine scale: %d ", sine_scale);

    Sine_set_amplitude(MAX_SINE_AMPLITUDE * (float)(sine_scale) / 100.0);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}


static const httpd_uri_t sine_scale_uri = {
    .uri       = "/sine_scale",
    .method    = HTTP_GET,
    .handler   = sine_scale_handler,
    .user_ctx  = NULL
};


// -----------------------------------------------
// SET CMD HANDLER
// -----------------------------------------------

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _value[32];

    if (parse_get(req, &buf) == ESP_OK)
    {
        if( httpd_query_key_value(buf, "startInv", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            ESP_LOGI(TAG, "Command StartInv: %d", value);

            startInv = (uint8_t)(value);
            UART_set_StartInv(startInv);
        } 
        else if( httpd_query_key_value(buf, "startAB", _value, sizeof(_value)) == ESP_OK ) 
        {
            int value = atoi(_value);
            ESP_LOGI(TAG, "Command startAB: %d", value);

            startAB = (uint8_t)(value);
            UART_set_StartAB(startAB);
        }
    }
    else 
    {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    free(buf);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}


static const httpd_uri_t cmd_uri = {
    .uri       = "/cmd",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
};



// -----------------------------------------------
// SET PWM HANDLER
// -----------------------------------------------


static esp_err_t pwm_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _pwm[32];

    if (parse_get(req, &buf) != ESP_OK ||
            httpd_query_key_value(buf, "pwm", _pwm, sizeof(_pwm)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int pwm = atoi(_pwm);
    ESP_LOGI(TAG, "Set PWM: %d", pwm);

    UART_set_Iset_level((uint8_t)(pwm));

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}


static const httpd_uri_t pwm_uri = {
    .uri       = "/pwm",
    .method    = HTTP_GET,
    .handler   = pwm_handler,
    .user_ctx  = NULL
};


// -----------------------------------------------
//
// -----------------------------------------------

/**
 * @brief Starts Invertor in soft mode
 * 
 */
void StartInvertor(void)
{
    for(float scale = MIN_SINE_AMPLITUDE; scale < (MAX_SINE_AMPLITUDE * 0.9); scale += 1.0)
    {
        Sine_set_amplitude(scale);
        vTaskDelay(2);
    }

}

// -----------------------------------------------
// SERVER
// -----------------------------------------------


static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.lru_purge_enable = true;
    config.core_id = 0;     // Start HTTP server on Core 0

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &status);
        httpd_register_uri_handler(server, &sine_scale_uri);
        httpd_register_uri_handler(server, &pwm_uri);
        httpd_register_uri_handler(server, &cmd_uri);
        httpd_register_uri_handler(server, &clear_fault_uri);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

/**
 * @brief WiFi disconnect event handler
 * 
 * @param arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {

        // Fix for WiFi display disconnect
        //ESP_LOGI(TAG, "Stopping webserver");
        //stop_webserver(*server);
        //*server = NULL;
    }
}

/**
 * @brief WiFi Connect handler
 * 
 * @param arg 
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}



/**
 * @brief Main 
 * 
 */
void app_main(void)
{
    // Server initialization

    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(wifi_init_softap());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &disconnect_handler, &server));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    //ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    server = start_webserver();

    GPIO_init();
    pwm_init();
    Sine_start_task();
    ADC_start_task();
    UART_start_task();
    enc28j60_init();

    // System state initialization
    startInv = FALSE;
    startAB = FALSE;
    UART_set_StartInv(startInv);
    UART_set_StartAB(startAB);


    // Sine Soft Start
    for(float scale = MIN_SINE_AMPLITUDE; scale < (MAX_SINE_AMPLITUDE * 0.9); scale += 1.0)
    {
        Sine_set_amplitude(scale);
        vTaskDelay(2);
    }


    while(1)
    {
        if(http_response_active == FALSE)
        {
            ADC_get_values(&system_params);
            ESP_LOGD(TAG, "Uab = %d", system_params.Uab);
        }

        vTaskDelay(60);
    }

}
