
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
#include "nvs_flash.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 27
#define MAX_APs 20


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
//        esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
//        ESP_LOGI(TAG, "hello world!");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(110 / portTICK_PERIOD_MS);
    }
}

void print_to_host(char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	esp_apptrace_vprintf(msg, args);
	va_end(args);
	esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
}

static char* getAuthModeName(wifi_auth_mode_t auth_mode) {

	char *names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "MAX"};
	return names[auth_mode];
}

void app_main()

{
	esp_log_set_vprintf(esp_apptrace_vprintf);
	ESP_ERROR_CHECK(nvs_flash_init());
	tcpip_adapter_init();
	ESP_ERROR_CHECK ( esp_event_loop_init ( event_handler , NULL ) ) ;
	esp_event_loop_init(event_handler, NULL);
	wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT ( ) ;
	ESP_ERROR_CHECK ( esp_wifi_init ( & wifi_config ) ) ;
	ESP_ERROR_CHECK ( esp_wifi_set_mode ( WIFI_MODE_STA ) ) ;
	ESP_ERROR_CHECK ( esp_wifi_start ( ) ) ;
	wifi_scan_config_t scan_config =  {
		. ssid  =  0 ,
		. bssid  =  0 ,
		. channel  =  0 ,
	        . show_hidden  =  true
	} ;
	ESP_ERROR_CHECK ( esp_wifi_scan_start ( & scan_config ,  true ) );
	uint16_t ap_num = MAX_APs;
	wifi_ap_record_t ap_records [ MAX_APs ];
	ESP_ERROR_CHECK ( esp_wifi_scan_get_ap_records ( & ap_num , ap_records ) );
	print_to_host("Found %d access points:\n", ap_num);
		print_to_host("\n");
		print_to_host("               SSID              | Channel | RSSI |   Auth Mode \n");
		print_to_host("----------------------------------------------------------------\n");
		for(int i = 0; i < ap_num; i++)
			print_to_host("%32s | %7d | %4d | %12s\n", (char*)ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi, getAuthModeName(ap_records[i].authmode));
		print_to_host("----------------------------------------------------------------\n");
//	esp_apptrace_init();
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 10, NULL);
}
