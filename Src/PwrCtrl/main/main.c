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

// ===================================================================
// LOCAL VARIABLES
// ===================================================================

static const char *TAG = "example";
int outputPinState = 1;
volatile int http_response_active = FALSE;

ParamDataFrame_t system_params;

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


/**
 * @brief 
 * 
 * @param buf 
 * @param str 
 * @return int 
 */
static int println(char *buf, char * str)
{
    int len = sprintf(buf, str);

    buf += len;
    sprintf(buf, "\r\n");

    return len + 2;
}

/**
 * @brief 
 * 
 * @param buf 
 * @param str 
 * @return int 
 */
static int print(char *buf, char * str)
{
    return sprintf(buf, str);
}

/* An HTTP GET handler */
static esp_err_t index_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    http_response_active = TRUE;
    // --------------------- Send html page as a response -----------------------

    LEDstate(outputPinState);

    char resp_buf[1024];
    char *resp = resp_buf;
    resp[0] = '\0';
    
    ESP_LOGI(TAG, "URI = %s; Method = %d", &(req->uri[0]), req->method);

    
    resp += println(resp, "<!DOCTYPE html><html>");
    resp += println(resp, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    resp += println(resp, "<meta charset=\"utf-8\">");    
    resp += println(resp, "<link rel=\"icon\" href=\"data:,\">");

    // CSS to style the on/off buttons 
    // Feel free to change the background-color and font-size attributes to fit your preferences
    resp += println(resp, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
    resp += println(resp, ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
    resp += println(resp, "text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
    resp += println(resp, ".button2 {background-color: #e34234;}</style></head>");
            
    // Web Page Heading
    resp += println(resp, "<body><h1>Состояние системы электропитания</h1>");
            
    // Display current state, and ON/OFF buttons for GPIO 26  
    resp += print(resp, "<p>Состояние инвертора: ");
    if (outputPinState == 0)
        resp += print(resp, "Остановлен");
    else
        resp += print(resp, "Включен");

    resp += println(resp, "</p>");

    resp += sprintf(resp, "</p>Uab = %d,%d В</p>\r\n", system_params.Uab/1000, system_params.Uab % 1000);

    // If the outputPinState is off, it displays the ON button       
    if (outputPinState==0) 
    {
        resp += println(resp, "<p><a href=\"/hello\"><button class=\"button\">START</button></a></p>");
        outputPinState = 1;
    } 
    else 
    {
        resp += println(resp, "<p><a href=\"/hello\"><button class=\"button button2\">STOP</button></a></p>");
        outputPinState = 0;
    } 
               
    resp += println(resp, "</body></html>");
            
    // The HTTP response ends with another blank line
    resp += println(resp, " \r\n");
    *resp++ = '\0';

    //----------------------------- End of html page response -------------------------------------
    http_response_active = FALSE;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }


    /* Read URL query string length and allocate memory for length + 1,
     * +1 extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) 
    {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) 
        {
            ESP_LOGI(TAG, "Found URL query => %s", buf);

            char param[32];
            
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    
    ESP_LOGI(TAG, "Sending response: %d bytes", strlen(resp_buf));

    ESP_LOGI(TAG, "%s", resp_buf);
    httpd_resp_send(req, resp_buf, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    
    return ESP_OK;
}


static const httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL // helloReply
};


// -----------------------------------------------
// STATUS HANDLER
// -----------------------------------------------

/* Status GET handler */
static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    char *p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"u_seti\":%u.%u,", system_params.Useti/1000, system_params.Useti % 1000);
    p += sprintf(p, "\"u_inv\":%u.%u,", system_params.Uinv/1000, system_params.Uinv % 1000);
    p += sprintf(p, "\"i_te\":%u.%u,", system_params.Ite/1000, system_params.Ite % 1000);
    p += sprintf(p, "\"u_te\":%u.%u,", system_params.Ute/1000, system_params.Ute % 1000);
    p += sprintf(p, "\"u_ab\":%u.%u,", system_params.Uab/1000, system_params.Uab % 1000);
    p += sprintf(p, "\"i_ab\":%u.%u,", system_params.Iab/1000, system_params.Iab % 1000);   
    p += sprintf(p, "\"batteries\":%s,", "\"1111 0101 1011\"");
    p += sprintf(p, "\"start_inv\":%u,", 1);
    p += sprintf(p, "\"start_ab\":%u", 0);

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
// PAGE HANDLER
// -----------------------------------------------



/* Page from file index.html
 */
static esp_err_t ctrl_handler(httpd_req_t *req)
{
    extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
    extern const unsigned char index_html_gz_end[] asm("_binary_index_html_gz_end");
    size_t index_html_gz_len = index_html_gz_end - index_html_gz_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_len);
}

static const httpd_uri_t ctrl_uri = {
    .uri       = "/ctrl",
    .method    = HTTP_GET,
    .handler   = ctrl_handler,
    .user_ctx  = NULL
};


// -----------------------------------------------
// SET PWM HANDLER
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
    ESP_LOGI(TAG, "Set Sine scale: %d ", sine_scale);


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
// SET SINE SCALE HANDLER
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
        httpd_register_uri_handler(server, &ctrl_uri);
        httpd_register_uri_handler(server, &sine_scale_uri);
        httpd_register_uri_handler(server, &pwm_uri);
        
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

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}


void app_main(void)
{
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
