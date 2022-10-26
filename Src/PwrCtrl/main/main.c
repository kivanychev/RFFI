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

// ===================================================================
// CONSTANTS
// ===================================================================


#define ON      1
#define OFF     0
#define TRUE    1
#define FALSE   0
#define BAT_OK      '+'
#define BAT_FAILURE '-'

#define ISET_DEFAULT        0xff

#define U_SETI_THRESHOLD    (220000 * 0.9)

#define ALL_BATTERIES_OK    0
#define INVERTER_FAULT      0
#define MAX_AB_CURRENT      6000
#define SOFT_START_STEP     2.5

// ===================================================================
// LOCAL VARIABLES
// ===================================================================

static const char *TAG = "MAIN";

// Indicates that http response is received and will not mess parameters data
volatile int http_response_active = FALSE;

// System parameters
ParamDataFrame_t params;
ADC_coeff_t *param_coeff = NULL;

// Battery sections states
volatile uint8_t st_batteries[12] = {BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK, 
                                  BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK, BAT_OK};

// Variables for controlling Inverter
volatile uint8_t ctrl_startInv = FALSE;         // Starts/Stops Inverter. Assigned with webpage control
volatile uint8_t ctrl_startAB = FALSE;          // Starts/Stops Charging device. Assigned with webpage control
volatile uint8_t ctrl_manualMode = FALSE;       // TRUE - manual mode; FALSE - automatic mode

// Power controller element states
volatile uint8_t state_InvStarted = FALSE;      // Indicates if the Invertor has finished soft start step or not
volatile uint8_t state_InvFault = FALSE;        // Indicates if Inverter driver Fault signal received
volatile uint8_t state_ABsectionsGood = TRUE;   // Indicates tha all sections of AB are good

// Sine amplitude value for controlling sine scale in soft start and stabilization modes
volatile float sine_amplitude = MIN_SINE_AMPLITUDE;

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
 * @brief Enables or disables Inverter sine wave generation 
 *        and sets Start_Inv signal to 1 or 0 depending on 'value'
 *        After the Inveter stops a new softStart procedure will be performed on next start
 * 
 * @param value     ON, OFF
 */
void Set_StartInv(uint8_t value)
{
    ctrl_startInv = value;
    UART_set_StartInv(ctrl_startInv);

    // Wait for the signal to pass to Atmega over UART
    vTaskDelay(1);


    if(value == OFF)
    {
        ESP_LOGI(TAG, "Sine_stop_wave()");
        Sine_stop_wave();

        // This flag will be set after soft start procedure is finished in the main code
        state_InvStarted = FALSE;
        sine_amplitude = MIN_SINE_AMPLITUDE;
    }
    else {
        ESP_LOGI(TAG, "Sine_start_wave()");
        Sine_start_wave();
    }
}

/**
 * @brief Enables or disables Start_AB signal
 * 
 * @param value     ON, OFF
 */
void Set_StartAB(uint8_t value)
{
    ctrl_startAB = value;

    UART_set_StartAB(ctrl_startAB);
}

// -----------------------------------------------
// STATUS HANDLER
// -----------------------------------------------


/* Status GET handler */
static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    uint16_t bat_state = UART_get_battery_state();

    // Reading st_batteries state into string form
    for(int ind_b = 0; ind_b < 12; ++ind_b)
    {
        if( (bat_state & (1 << ind_b)) == 0) 
        {
            st_batteries[ind_b] = BAT_OK;
        }
        else
        {
            st_batteries[ind_b] = BAT_FAILURE;
        }
    }

    char *p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"u_seti\":%u.%u,", params.Useti/1000, params.Useti % 1000);
    p += sprintf(p, "\"u_inv\":%u.%u,", params.Uinv/1000, params.Uinv % 1000);
    p += sprintf(p, "\"i_te\":%f,", (float)(params.Ite)/1000.0);
    p += sprintf(p, "\"u_te\":%u.%u,", params.Ute/1000, params.Ute % 1000);
    p += sprintf(p, "\"u_ab\":%u.%u,", params.Uab/1000, params.Uab % 1000);
    p += sprintf(p, "\"i_ab\":%f,", (float)(params.Iab)/1000.0);
    p += sprintf(p, "\"batteries\":\"%c%c%c%c %c%c%c%c %c%c%c%c\",", st_batteries[0], st_batteries[1], st_batteries[2],
                                                                     st_batteries[3], st_batteries[4], st_batteries[5],
                                                                     st_batteries[6], st_batteries[7], st_batteries[8],
                                                                     st_batteries[9], st_batteries[10], st_batteries[11]);
    p += sprintf(p, "\"invertor-state-text\":%s,", UART_get_fault_state() == 1 ? "\"Норма\"" : "\"Ошибка\"" );
    p += sprintf(p, "\"eth-ip-text\":\"%s\",", ENC28J60_getEthernetIP());

    p += sprintf(p, "\"start_inv\":%u,", ctrl_startInv);
    p += sprintf(p, "\"manual-mode\":%u,", ctrl_manualMode);

    p += sprintf(p, "\"start_ab\":%u", ctrl_startAB);

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

            // Server command should work for manual mode
            if(ctrl_manualMode == TRUE)
            {
                Set_StartInv((uint8_t)(value));
            }
        } 
        else if( httpd_query_key_value(buf, "startAB", _value, sizeof(_value)) == ESP_OK ) 
        {
            int value = atoi(_value);
            ESP_LOGI(TAG, "Command StartAB: %d", value);

            ctrl_startAB = (uint8_t)(value);
            UART_set_StartAB(ctrl_startAB);
        }
        else if( httpd_query_key_value(buf, "manualMode", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            ctrl_manualMode = (uint8_t)(value);

            ESP_LOGI(TAG, "Command ManualMode: %d", ctrl_manualMode);

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
// GET COEFFICIENTS HANDLER
// -----------------------------------------------

/* get_coeffs handler */
static esp_err_t get_coeffs_handler(httpd_req_t *req)
{
    static char json_response[1024];

    char *p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"u-ab-coeff\":%u,", param_coeff->U_AB);
    p += sprintf(p, "\"u-inv-coeff\":%u,", param_coeff->U_INV);
    p += sprintf(p, "\"i-ab-coeff\":%u,", param_coeff->I_AB);
    p += sprintf(p, "\"i-te-coeff\":%u,", param_coeff->I_TE);
    p += sprintf(p, "\"u-seti-coeff\":%u,", param_coeff->U_SETI);
    p += sprintf(p, "\"u-te-coeff\":%u", param_coeff->U_TE);

    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}


static const httpd_uri_t get_coeffs_uri = {
    .uri       = "/get-coeffs",
    .method    = HTTP_GET,
    .handler   = get_coeffs_handler,
    .user_ctx  = NULL
};


// -----------------------------------------------
// SET COEFF HANDLER
// -----------------------------------------------


static esp_err_t set_coeff_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _value[32];

    if (parse_get(req, &buf) == ESP_OK)
    {
        if( httpd_query_key_value(buf, "u_seti_coeff", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            ESP_LOGI(TAG, "u_seti_coeff: %d", value);

            param_coeff->U_SETI = (uint16_t)(value);
            ADC_update_coeff();
        } 
        else if( httpd_query_key_value(buf, "u_inv_coeff", _value, sizeof(_value)) == ESP_OK ) 
        {
            int value = atoi(_value);
            ESP_LOGI(TAG, "u_inv_coeff: %d", value);

            param_coeff->U_INV = (uint16_t)(value);
            ADC_update_coeff();
        }
        else if( httpd_query_key_value(buf, "u_te_coeff", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            param_coeff->U_TE = (uint16_t)(value);

            ESP_LOGI(TAG, "u_te_coeff: %d", value);
            ADC_update_coeff();
        }
        else if( httpd_query_key_value(buf, "i_te_coeff", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            param_coeff->I_TE = (uint16_t)(value);

            ESP_LOGI(TAG, "i_te_coeff: %d", value);
            ADC_update_coeff();
        }
        else if( httpd_query_key_value(buf, "i_ab_coeff", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            param_coeff->I_AB = (uint16_t)(value);

            ESP_LOGI(TAG, "i_ab_coeff: %d", value);
            ADC_update_coeff();
        }
        else if( httpd_query_key_value(buf, "u_ab_coeff", _value, sizeof(_value)) == ESP_OK )
        {
            int value = atoi(_value);
            param_coeff->U_AB = (uint16_t)(value);

            ESP_LOGI(TAG, "u_ab_coeff: %d", value);
            ADC_update_coeff();
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


static const httpd_uri_t set_coeff_uri = {
    .uri       = "/set-coeff",
    .method    = HTTP_GET,
    .handler   = set_coeff_handler,
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
        httpd_register_uri_handler(server, &set_coeff_uri);
        httpd_register_uri_handler(server, &get_coeffs_uri);

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

    param_coeff = ADC_init_coeff();
    ADC_start_task();

    UART_start_task();
    enc28j60_init();

    // Variables reflecting the states of the UI controlling switches
    ctrl_startInv = FALSE;
    ctrl_startAB = FALSE;
    ctrl_manualMode = FALSE;

    // Temporayry states variables 
    uint8_t sig_startInv = FALSE;
    uint8_t sig_startAB = FALSE;

    sine_amplitude = MIN_SINE_AMPLITUDE;

    // Default signal states
    UART_set_StartInv(ctrl_startInv);
    UART_set_StartAB(ctrl_startAB);
    UART_set_Iset_level(ISET_DEFAULT);

    while(1)
    {
        if(http_response_active == FALSE)
        {
            ADC_get_values(&params);
            ESP_LOGD(TAG, "Useti = %d", params.Useti);
        }

        //////////////////////////
        // Control Automatic mode
        //////////////////////////

        if(ctrl_manualMode == FALSE)
        {
            // Check if the Inverter is to be turned On
            if(params.Useti < U_SETI_THRESHOLD ) 
            {
                //////////////////////////////////////////////
                //            FAILURE HANDLING                
                //////////////////////////////////////////////

                sig_startInv = TRUE;
                sig_startAB = TRUE;

                // Check batteries
                if( UART_get_battery_state() != ALL_BATTERIES_OK )
                {
                    if(params.Iab < 0)
                    {
                        sig_startAB = FALSE;
                    }
                    else
                    {
                        ESP_LOGD(TAG, "Bat error: Iab > 0 ---> Start_Inv = 0");
                        sig_startInv = FALSE;
                    }
                }

                // Check Inverter Fault signal
                if( UART_get_fault_state() == INVERTER_FAULT )
                {
                    ESP_LOGD(TAG, "Fault: ---> Start_Inv = OFF");
                    sig_startInv = FALSE;
                }

                // Turn off the Inverter if the AB voltage is lower than 300V
                if( params.Uab < 300000 )
                {
                    ESP_LOGD(TAG, "Uab < 300V  ---> Start_Inv = OFF");
                    sig_startInv = FALSE;
                }

                // ---------------------
                // APPLY INVERTER SIGNAL 
                // ---------------------

                if(sig_startInv == TRUE)
                {
                    ESP_LOGD(TAG, "sig_startInv == TRUE");
                    // If not yet started then indicate to start the Inverter
                    if(state_InvStarted == FALSE) {
                        ESP_LOGI(TAG, "Set Invertor to start as Useti < 0.9 * 220 V");

                        Set_StartInv(ON);
                    }
                }
                else
                {
                    ESP_LOGD(TAG, "sig_startInv == FALSE");
                    Set_StartInv(OFF);
                }

                // ---------------
                // APPLY AB SIGNAL 
                // ---------------

                Set_StartAB(sig_startAB);


                //////////////////////////////////////////////
                //           STABILIZATION HANDLING
                //////////////////////////////////////////////

                if( state_InvStarted == TRUE )
                {
                    if(params.Iab > MAX_AB_CURRENT)
                    {
                        ESP_LOGI(TAG, "Iab > 6A");
                        if(sine_amplitude > MIN_SINE_AMPLITUDE)
                        {
                            sine_amplitude -= 0.5;
                            Sine_set_amplitude(sine_amplitude);
                        }
                    }

                    if(params.Iab <= MAX_AB_CURRENT)
                    {
                        if(sine_amplitude < MAX_SINE_AMPLITUDE )
                        {
                            sine_amplitude += 0.5;
                            Sine_set_amplitude(sine_amplitude);
                            ESP_LOGD(TAG, "Iab < 5.8A, set amplitude %f", sine_amplitude);
                        }
                    }
                }

            }
            else 
            {   // Turn off the Inverter
                Set_StartInv(OFF);
                Set_StartAB(OFF);
            }
        }

        ////////////////////////////////////
        // Start Inverter if not yet started
        // For Manual and Automatic modes
        ////////////////////////////////////

        if(ctrl_startInv == TRUE)
        {
            if(state_InvStarted == FALSE)
            {
                // PERFORM INVERTER SOFT START HERE
                if( sine_amplitude <= MAX_SINE_AMPLITUDE && params.Iab <= MAX_AB_CURRENT)
                {
                    Sine_set_amplitude(sine_amplitude);
                    sine_amplitude += SOFT_START_STEP;
                }
                else
                {
                    sine_amplitude = sine_amplitude;
                    ESP_LOGI(TAG, "Inverter started");
                    state_InvStarted = TRUE;
                }
            }
        }

        vTaskDelay(5);
    }

}
