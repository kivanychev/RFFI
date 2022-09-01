/* General Purpose Timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"

#include "soc/mcpwm_periph.h"

#include <esp_log.h>

#include "pwm_control.h"
#include "sine_timer.h"

// ======================================================================
// CONSTANT DEFINITIONS
// ======================================================================

#define TIMER_DIVIDER           (16)                                //  Hardware timer clock divider
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER)    // convert counter value to seconds
#define SINE_SPEED              10
#define NVALUES                 400                                 // Number of SINE levels

#define FALSE                   0
#define TRUE                    1

// ======================================================================
//  TYPE DEFINITIONS
// ======================================================================

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;


/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

// ======================================================================
// LOCAL VARIABLES
// ======================================================================

static const char *TAG = "SINE TIMER";

static volatile uint8_t sine_enabled = FALSE;

static volatile float sine_scale = MIN_SINE_AMPLITUDE;    // 50 - maximum value - 100% (MAX_SINE_AMPLITUDE)
static xQueueHandle s_timer_queue;

// ======================================================================
// FUNCTION DEFINITIONS
// ======================================================================

/**
 * @brief ISR for Timer events
 * 
 * @param args - Used for passing event data to Queue
 * @return true 
 * @return false 
 */
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* Prepare basic event data that will be then sent back to task */
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    /* Now just send the event data back to the program task */
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}



/**
 * @brief Initialize timer for sine wave form generation
 *
 * @param group - Timer Group number, index from 0
 * @param timer - timer ID, index from 0
 * @param auto_reload - whether auto-reload on alarm event
 * @param timer_interval_millisec - Interval of sine period in milliseconds
 */
static void sine_timer_init(int group, int timer, bool auto_reload, int timer_interval_millisec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB

    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_millisec * (TIMER_SCALE / 1000) / NVALUES );
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_millisec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);
}


/**
 * @brief Task for generating sine form PWM
 * 
 * @param arg - Not used
 */
static void sine_timer_task(void *arg)
{
    double sine_value[NVALUES];

    int pos;
    int duty_a = 0;
    int duty_b = NVALUES / 2;

    /* Prepare sine values array */
    for(pos = 0; pos < NVALUES; ++pos)
    {
        sine_value[pos] = sin( (float)(pos) * 2.0 * M_PI / (float)(NVALUES) );
    }
    
    /* Create a queue for receiving data from ISR */
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));

    sine_timer_init(TIMER_GROUP_0, TIMER_0, true, 20);      // 20 millisec for sine period

    while (1) 
    {
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);

        // Fof the disables sine generation
        if(sine_enabled == FALSE)
        {
            duty_a = 0;
            duty_b = NVALUES / 2;
            sine_scale = 0;
            continue;
        }

        // Received message from timer here
        duty_a ++;
        duty_b ++;

        if(duty_a >= NVALUES)
        {
            duty_a = 0;
        }

        if(duty_b >= NVALUES)
        {
            duty_b = 0;
        }

        // Sine wave is generated in positive values by shifting it's values 
        // 50.0 is a 0-lelev for up-shisfted Sine wave
        // The whole Sine values range is 100: From -50 to 50
        // Moving Sine up by 50.0 will shift the whole Sine to positive values
        // This shift allows to calculate timer compare levels and map them to Timer value that is only in positive values

        set_duty_a( 50.0 + sine_scale * sine_value[duty_a]);
        set_duty_b( 50.0 + sine_scale * sine_value[duty_b]);
    }
}


// ======================================================================
// EXTERNAL FUNCTION DEFINITIONS
// ======================================================================

/**
 * @brief Stops Sine wave generation
 * 
 */
void Sine_stop_wave()
{
    sine_enabled = FALSE;

    ESP_LOGI(TAG, "Sine_stop_wave()");

    set_duty_a( 50.0);
    set_duty_b( 50.0);
}

/**
 * @brief Starts Sine wave generation
 * 
 */
void Sine_start_wave()
{
    ESP_LOGI(TAG, "Sine_start_wave()");

    sine_enabled = TRUE;
}


/**
 * @brief Set the amplitude value for Invertor output voltage
 * 
 * @param ampl - amplitude value between 0 and MAX_SINE_AMPLITUDE
 */
void Sine_set_amplitude(float ampl)
{
    if(ampl > MAX_SINE_AMPLITUDE)
    {
        sine_scale = MAX_SINE_AMPLITUDE;
    }

    sine_scale = ampl;
}

/**
 * @brief Start sine timer task
 * 
 */
void Sine_start_task(void)
{
    xTaskCreatePinnedToCore(sine_timer_task, "sine_timer_task", 8192, NULL, 2, NULL, 1);
}