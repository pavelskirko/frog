/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_app_trace.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 27


static const char* TAG = "Blink";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}


void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */

    	gpio_set_level(BLINK_GPIO, 0);
//        esp_log_set_vprintf(esp_apptrace_vprintf);
        ESP_LOGI(TAG, "12345");
        esp_log_set_vprintf(esp_apptrace_vprintf);
        esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(110 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
//	tcpip_adapter_init();
//	esp_event_loop_init(event_handler, NULL);
//	wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
//	esp_wifi_init(&wifi_config);
//	esp_wifi_set_mode(WIFI_MODE_STA);
//	esp_wifi_start();
//	wifi_scan_config_t scan_config = {
//		.ssid = 0,
//		.bssid = 0,
//		.channel = 0,
//	        .show_hidden = true
//	};
//	esp_wifi_scan_start(&scan_config, true);
//	uint16_t ap_num = 10;
//	wifi_ap_record_t ap_records[10];
//	esp_wifi_scan_get_ap_records(&ap_num, ap_records);

	esp_apptrace_init();
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 10, NULL);
}
