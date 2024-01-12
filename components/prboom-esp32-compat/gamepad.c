// Copyright 2016-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <stdlib.h>

#include "doomdef.h"
#include "doomtype.h"
#include "m_argv.h"
#include "d_event.h"
#include "g_game.h"
#include "i_joy.h"
#include "d_main.h"
#include "gamepad.h"
#include "lprintf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"

//The gamepad uses keyboard emulation, but for compilation, these variables need to be placed
//somewhere. This is as good a place as any.
int usejoystick=1;
int joyleft, joyright, joyup, joydown;


//atomic, for communication between joy thread and main game thread
volatile int joyVal=0;

typedef struct {
	int gpio;
	int *key;
	int64_t last_event_us;
} GPIOKeyMap;

typedef struct {
    int gpio;
    bool level;
    int64_t pressed_at_us;
} ButtonEvent;

//Mappings from PS2 buttons to keys
GPIOKeyMap keymap[]={
	{CONFIG_HW_BUTTON_DOWN_PIN,  &key_fire, 		0},			
	{CONFIG_HW_BUTTON_UP_PIN,    &key_strafe, 		0},			
	{CONFIG_HW_BUTTON_LEFT_PIN,  &key_use,	 	    0},
	{CONFIG_HW_BUTTON_RIGHT_PIN, &key_speed, 		0},
	{CONFIG_HW_BUTTON_JOY_PIN,   &key_menu_enter, 	0},
};
const int num_keys = sizeof(keymap) / sizeof(GPIOKeyMap);

/*	
	{0x2000, &key_menu_enter},		//circle
	{0x8000, &key_pause},			//square
	{0x1000, &key_weapontoggle},	//triangle

	{0x8, &key_escape},				//start
	{0x1, &key_map},				//select
	
	{0x400, &key_strafeleft},		//L1
	{0x100, &key_speed},			//L2
	{0x800, &key_straferight},		//R1
	{0x200, &key_strafe},			//R2

	{0, NULL},
};
*/

void gamepadPoll(void)
{
}

static QueueHandle_t gpio_evt_queue = NULL;

static adc_oneshot_unit_handle_t adc_handle;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    int gpio_num = (int) arg;
    bool level = gpio_get_level(gpio_num) != 0;
    int64_t time_us = esp_timer_get_time();

    ButtonEvent event = {
        .gpio = gpio_num,
        .level = level,
        .pressed_at_us = time_us,
    };

    xQueueSendFromISR(gpio_evt_queue, &event, NULL);
}


void gpioTask(void *arg) {
	lprintf(LO_INFO, "GPIO task running...\n");

    ButtonEvent event;
	event_t ev;

    while(true) {
        TickType_t timeout = CONFIG_HW_JOYSTICK_SAMPLE_INTERVAL_US / portTICK_PERIOD_MS;

        // === sample buttons ===
        if(xQueueReceive(gpio_evt_queue, &event, timeout)) {
			for (int i=0; i < num_keys; i++) {
                GPIOKeyMap *cur_map = &keymap[i];

				if(cur_map->gpio == event.gpio)
				{
                    // debounce TODO improve
                    int64_t tdiff_us = event.pressed_at_us - cur_map->last_event_us;

                    if(tdiff_us > CONFIG_HW_DEBOUNCE_THRESHOLD_US) {
                        cur_map->last_event_us = event.pressed_at_us;

                        printf("key: %d, level: %d, tdiff: %d\n", *cur_map->key, event.level, (int)tdiff_us);

                        // buttons are pulled low
                        ev.type = event.level ? ev_keyup : ev_keydown;
                        ev.data1 = *cur_map->key;
                        D_PostEvent(&ev);

                        // todo implement a button state confirm a few ms later or something
                    }
				}
			}
        }

        // === sample joystick ===
        int32_t x_raw, y_raw;

        // read 0-3.3V -> 12 bit unsigned
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &x_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &y_raw));

        int x_val = (x_raw >> 4) - 127;
        int y_val = (y_raw >> 4) - 127;

        int x_dir = 0, y_dir = 0;

        if (x_val > CONFIG_HW_JOYSTICK_DEADZONE) x_dir = 1;
        if (x_val < -CONFIG_HW_JOYSTICK_DEADZONE) x_dir = -1;
        if (y_val > CONFIG_HW_JOYSTICK_DEADZONE) y_dir = -1;
        if (y_val < -CONFIG_HW_JOYSTICK_DEADZONE) y_dir = 1;

        ev.type = ev_joystick;
        ev.data1 = 0;
        ev.data2 = x_dir;
        ev.data3 = y_dir;
        D_PostEvent(&ev);

        // printf("Joystick X: %d, Y: %d\n", x_val, y_val);
    }
}

void jsInit(void) 
{
	lprintf(LO_INFO, "gamepad init...\n");

    // == init button interrupt handlers ==
	gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = 0,
    };

    //bit mask the pins
	for (int i=0; i < num_keys; i++) {
		io_conf.pin_bit_mask |= (1ULL<<keymap[i].gpio);
	}
	
    gpio_config(&io_conf);


    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(ButtonEvent));
    //start gpio task
	xTaskCreatePinnedToCore(&gpioTask, "GPIO", 4096, NULL, 7, NULL, 1); // TODO is core 1 suitable?

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_SHARED);
    //hook isr handler for specific gpio pin
	for (int i=0; i < num_keys; i++) {
    	gpio_isr_handler_add(keymap[i].gpio, gpio_isr_handler, (void*) keymap[i].gpio);
	}

    // == init adc for joystick ==
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    // x-axis on GPIO 1
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &config));
    // y-axis on GPIO 2
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &config));

	lprintf(LO_INFO, "gamepad init: GPIO task created.\n");
}
