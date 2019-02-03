#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "udp_logging.h"
#include "mssg.pb.h"
#include "tcp_server.h"
#include "spi.h"

static const char *TAG = "frog";

void ok_blink_task(void *pvParameters)
{
	int level1 = 0;
	while (1)
	{
		    	gpio_set_level(27, level1);
		    	ESP_LOGI(TAG, "TICK");
		    	level1 = !level1;
		    	vTaskDelay(300 / portTICK_PERIOD_MS);
	}

}


void app_main(void)
{
		esp_err_t ret = nvs_flash_init();
	    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	    {
	      ESP_ERROR_CHECK(nvs_flash_erase());
	      ret = nvs_flash_init();
	    }
	    ESP_ERROR_CHECK(ret);
	    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
	    gpio_set_direction(27, GPIO_MODE_OUTPUT);
	    gpio_set_direction(26, GPIO_MODE_OUTPUT);
	    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	    wifi_event_group = xEventGroupCreate();
	    start_dhcp_server();
	    initialise_wifi_in_ap();
//	    udp_logging_init( "192.168.1.2", 1337, udp_logging_vprintf);
	    xTaskCreate(tcp_server,"tcp_server",4096,NULL,5,NULL);
//	    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
	    xTaskCreate(ok_blink_task, "ok_blink", 4096, NULL, 5, NULL);
	    xTaskCreate(get_data, "get_data", 8096, NULL, 5, NULL);

}

