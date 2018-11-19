
/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h> /* memset */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_app_trace.h"
#include "esp_log.h"
#include "esp_err.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "ICM-20602.h"
#include "AD-7797BRUZ.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 27
#define MAX_APs 20
#define WIFI_SSID "Home-5GHz"
#define WIFI_PASS "SlavaUkraini"

#define PIN_NUM_VSPI_Q 19 //miso
#define PIN_NUM_VSPI_D 23 //mosi
#define PIN_NUM_CLK  18
#define PIN_NUM_CS0   5
#define PIN_NUM_CS1   4
#define PIN_NUM_CS2   2

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static const char* TAG = "Blink";

void get_X(spi_device_handle_t * spi);
void print_to_host(char * msg, ...);
void spi_setup(spi_device_handle_t * spi);
void wifi_setup();

char data[100];
uint i_data = 100;
uint8_t who_am_i[126];
uint8_t ad_rec;

void print_to_host(char *msg, ...)
{
	va_list args;
	va_start(args, msg);
	esp_apptrace_vprintf(msg, args);
	va_end(args);
	esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	print_to_host("smth happened");
    switch(event->event_id) {

    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;

	case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;

	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;

	default:
        break;
    }

	return ESP_OK;
}

void spi_setup(spi_device_handle_t * spi)
{
	spi_bus_config_t buscfg={
	        .miso_io_num=PIN_NUM_VSPI_Q,
	        .mosi_io_num=PIN_NUM_VSPI_D,
	        .sclk_io_num=PIN_NUM_CLK,
	        .quadwp_io_num=-1,
	        .quadhd_io_num=-1,
	        .max_transfer_sz=4094,
	    };
	spi_device_interface_config_t devcfg0={
	        .clock_speed_hz=10*1000,           //Clock out at 10 MHz
	        .command_bits=0,
			.address_bits=32,
			.mode=0,                                //SPI mode 0
	        .spics_io_num=PIN_NUM_CS0,               //CS pin
	        .queue_size=7,                          //We want to be able to queue 7 transactions at a time  //Specify pre-transfer callback to handle D/C line
	    };

	ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg0, spi));
}



void accel_init(spi_device_handle_t * spi1)
{
//	spi_transaction_t trans={
//			            .addr=ICM20602_PWR_MGMT_1,
//			            .cmd=0x00,
//			            .flags=SPI_TRANS_USE_TXDATA,
//			            .length=8,
//			            .rxlength=8,
//			            .tx_buffer=0,
//						.tx_data=0x4, //cycle mode off
//			            .rx_buffer=0,
//			    };
//	ESP_ERROR_CHECK(spi_device_transmit(*spi1, &trans));
//	spi_transaction_t trans={
//			            .addr=ICM20602_PWR_MGMT_2,
//			            .cmd=0x00,
//			            .flags=SPI_TRANS_USE_TXDATA,
//			            .length=8,
//			            .rxlength=8,
//			            .tx_buffer=0,
//						.tx_data=0x00, //cycle mode on
//			            .rx_buffer=0,
//			    };
//	ESP_ERROR_CHECK(spi_device_transmit(*spi1, &trans));
//	spi_transaction_t trans={
//			            .addr=ICM20602_PWR_MGMT_2,
//			            .cmd=0x00,
//			            .flags=SPI_TRANS_USE_TXDATA,
//			            .length=8,
//			            .rxlength=8,
//			            .tx_buffer=0,
//						.tx_data=0x20, //cycle mode on
//			            .rx_buffer=0,
//			    };
}

void adc_setup(spi_device_handle_t * spi2)
{
	spi_device_interface_config_t devcfg2={
						        .clock_speed_hz=10*1000,           //Clock out at 10 MHz
						        .command_bits=8,
								.address_bits=8,
								.mode=0,                                //SPI mode 0
						        .spics_io_num=PIN_NUM_CS2,               //CS pin
						        .queue_size=7,  	//We want to be able to queue 7 transactions at a time
								.flags=0,//
						    };
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg2, spi2));
	spi_transaction_t trans;
			memset(&trans, 0, sizeof(spi_transaction_t));
			trans.tx_data[0] = (0x7C & AD_7797_MODE); // write to MODE
			trans.tx_data[1] = 0;
			trans.tx_data[2] = 0xF; // slowest update rate
			trans.length = 24;
			trans.flags = SPI_TRANS_USE_TXDATA;
			ESP_ERROR_CHECK(spi_device_queue_trans(*spi2, &trans, portMAX_DELAY));
			spi_transaction_t * r_trans;
			ESP_ERROR_CHECK(spi_device_get_trans_result(*spi2, &r_trans, portMAX_DELAY));
			memset(&trans, 0, sizeof(spi_transaction_t));
			trans.tx_data[0] = (0x7C & AD_7797_CONFG); // write to configuration
			trans.tx_data[1] = (1<<3) | 0x3; // unipolar
			trans.tx_data[2] = 0xF;
			trans.length = 24;
			trans.flags = SPI_TRANS_USE_TXDATA;
			ESP_ERROR_CHECK(spi_device_queue_trans(*spi2, &trans, portMAX_DELAY));
			ESP_ERROR_CHECK(spi_device_get_trans_result(*spi2, &r_trans, portMAX_DELAY));


}

void get_data(void *pvParameter)
{
	spi_device_handle_t spi1;
	spi_device_handle_t spi2;
	spi_device_handle_t spi3;
//	spi_init(&spi1);

	uint8_t low_b = 0;
	uint8_t high_b = 0;
	spi_bus_config_t buscfg={
				        .miso_io_num=PIN_NUM_VSPI_Q,
				        .mosi_io_num=PIN_NUM_VSPI_D,
				        .sclk_io_num=PIN_NUM_CLK,
				        .quadwp_io_num=-1,
				        .quadhd_io_num=-1,
				        .max_transfer_sz=4094,
				    };
	spi_device_interface_config_t devcfg={
				        .clock_speed_hz=2*1000*1000,           //Clock out at 2 MHz
				        .command_bits=0,
						.address_bits=8,
						.mode=0,                                //SPI mode 0
				        .spics_io_num=PIN_NUM_CS0,               //CS pin
				        .queue_size=7,  	//We want to be able to queue 7 transactions at a time
						.flags=0,//SPI_DEVICE_POSITIVE_CS,
				    };
	ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi1));
	devcfg.spics_io_num = PIN_NUM_CS1;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi2));
	devcfg.spics_io_num = PIN_NUM_CS2;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi3));
//	spi_transaction_t trans = {
//					            .addr=ICM20602_PWR_MGMT_1,
//					            .cmd=0,
//					            .flags=SPI_TRANS_USE_TXDATA,
//					            .length=64,
//					            .rxlength=8,
//					            .tx_buffer=0,
//								.tx_data=0x4, //cycle mode on
//					            .rx_buffer=0,
//					    };
	uint8_t tx_data = 0x20;
	spi_transaction_t trans[2];
	memset(&trans[0], 0, sizeof(spi_transaction_t));
	trans[0].addr = (0x80 | ICM20602_WHO_AM_I);
//	trans[0].cmd = 1;
	trans[0].length = 16;
//	memset(&trans[1], 0, sizeof(spi_transaction_t));
//	trans[0].length = 8;
	trans[0].rxlength=8;
	trans[0].rx_data[0] = 1;
	trans[0].rx_data[1] = 0;
	trans[0].rx_data[2] = 0;
	trans[0].rx_data[3] = 0;
	trans[0].flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(spi1, &trans[0], portMAX_DELAY));
//	ESP_ERROR_CHECK(spi_device_queue_trans(spi1, &trans[1], portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(spi1, &r_trans, portMAX_DELAY));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(spi1, &r_trans, portMAX_DELAY));
	who_am_i[0] = r_trans->rx_data[0];
//	who_am_i[1] = r_trans->rx_data[1];
//	who_am_i[2] = r_trans->rx_data[2];
//	who_am_i[3] = r_trans->rx_data[3];


//	{
//	            .addr=ICM20602_PWR_MGMT_1,
//	            .cmd=0,
//	            .flags=0,
//	            .length=16,
//	            .rxlength=0,
//	            .tx_buffer=&tx_data,
//	            .rx_buffer=0,
//	    };
////	ESP_ERROR_CHECK(spi_device_transmit(spi1, &trans));

//	trans.addr=ICM20602_WHO_AM_I;
//	trans.cmd=0;
//	trans.flags=0;//SPI_DEVICE_TXBIT_LSBFIRST,
//	trans.length=24;
//	trans.rxlength=8;
//	trans.tx_buffer=0;
//	trans.rx_buffer=&who_am_i;
////	ESP_ERROR_CHECK(spi_device_transmit(spi1, &trans));
//	ESP_ERROR_CHECK(spi_device_queue_trans(spi1, &trans, portMAX_DELAY));
//	spi_transaction_t * r_trans;


    while(1)
    {

    }
//    {	for (int i = 1; i < 126; i++)
//    	{
//    		trans[0].addr = (0x80 | i);
//    		ESP_ERROR_CHECK(spi_device_queue_trans(spi1, &trans[0], portMAX_DELAY));
//    		ESP_ERROR_CHECK(spi_device_get_trans_result(spi1, &r_trans, portMAX_DELAY));
//    		who_am_i[i] =  r_trans->rx_data[0];
//    	}

//    	get_X(p);
//    	print_to_host("Reading data...");
//    	trans.addr=ICM20602_ACCEL_XOUT_L;
//    	trans.rx_buffer=&low_b;
//    	ESP_ERROR_CHECK(spi_device_transmit(spi1, &trans));
//    	trans.addr = ICM20602_ACCEL_XOUT_H;
//    	trans.rx_buffer = &high_b;
//    	ESP_ERROR_CHECK(spi_device_transmit(spi1, &trans));
//    	vTaskDelay(100 / portTICK_PERIOD_MS);
//    	i_data = low_b | (high_b<<8);
//    	sprintf(data,"X = %u", i_data);
//    	sprintf(data,"X = %i", i_data);
//    	itoa(i_data, data, 10);
//    	print_to_host(data);
//    }

}

void get_data_adc(void *pvParameter)
{
	spi_device_handle_t spi2;
//	spi_bus_config_t buscfg={
//					        .miso_io_num=PIN_NUM_VSPI_Q,
//					        .mosi_io_num=PIN_NUM_VSPI_D,
//					        .sclk_io_num=PIN_NUM_CLK,
//					        .quadwp_io_num=-1,
//					        .quadhd_io_num=-1,
//					        .max_transfer_sz=4094,
//					    };
//		spi_device_interface_config_t devcfg={
//					        .clock_speed_hz=10*1000,           //Clock out at 10 MHz
//					        .command_bits=8,
//							.address_bits=8,
//							.mode=0,                                //SPI mode 0
//					        .spics_io_num=PIN_NUM_CS2,               //CS pin
//					        .queue_size=7,  	//We want to be able to queue 7 transactions at a time
//							.flags=0,//SPI_DEVICE_POSITIVE_CS,
//					    };
//		ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
//		ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi2));
		adc_setup(&spi2);
		spi_transaction_t trans[2];
		memset(&trans[0], 0, sizeof(spi_transaction_t));
		trans[0].tx_data[0] = (0x7C & ((1<<6) | AD_7797_ID )); // read id
		trans[0].length = 8;
		trans[0].flags = SPI_TRANS_USE_TXDATA;
		memset(&trans[1], 0, sizeof(spi_transaction_t));
		trans[1].rxlength=8;
		trans[1].length = 8;
		trans[1].rx_data[0] = 0;
		trans[1].flags = SPI_TRANS_USE_RXDATA;
		ESP_ERROR_CHECK(spi_device_queue_trans(spi2, &trans[0], portMAX_DELAY));
		ESP_ERROR_CHECK(spi_device_queue_trans(spi2, &trans[1], portMAX_DELAY));
		spi_transaction_t * r_trans;
		ESP_ERROR_CHECK(spi_device_get_trans_result(spi2, &r_trans, portMAX_DELAY));
		ESP_ERROR_CHECK(spi_device_get_trans_result(spi2, &r_trans, portMAX_DELAY));
		//	ESP_ERROR_CHECK(spi_device_get_trans_result(spi1, &r_trans, portMAX_DELAY));
		ad_rec = r_trans->rx_data[0];
}

void get_X(spi_device_handle_t * spi)
{
	uint8_t low_b, high_b;
	spi_transaction_t trans={
	            .addr=ICM20602_ACCEL_XOUT_L,
	            .cmd=0,
	            .flags=0,
	            .length=8,
	            .rxlength=8,
	            .tx_buffer=0,
	            .rx_buffer=&low_b,
//				.rx_data=&low_b,
	    };
	ESP_ERROR_CHECK(spi_device_transmit(*spi, &trans));
	trans.addr = ICM20602_ACCEL_XOUT_H;
	trans.rx_buffer = &high_b;
//	spi_transaction_t trans={
//		            .addr=ICM20602_ACCEL_XOUT_H,
//		            .cmd=0,
//		            .flags=0,
//		            .length=0,
//		            .rxlength=8,
//		            .tx_buffer=0,
//		            .rx_buffer=&high_b,
//		    };
	ESP_ERROR_CHECK(spi_device_transmit(*spi, &trans));
	int i_data = low_b | (high_b<<8);
	char data[2];
	sprintf(data,"%d", i_data);
	print_to_host(data);
}

void blink_task(void *pvParameter)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */

    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */

    	gpio_set_level(BLINK_GPIO, 0);
//        esp_log_set_vprintf(esp_apptrace_vprintf);
//        esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
//        ESP_LOGI(TAG, "hello world!");
//    	sprintf(data,"X = %i", 1);
//
//    	print_to_host(data);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(110 / portTICK_PERIOD_MS);
    }
}



static char* getAuthModeName(wifi_auth_mode_t auth_mode) {

	char *names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "MAX"};
	return names[auth_mode];
}

void wifi_task(void *pvParameter)
{
	// wait for connection
	print_to_host("Main task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	print_to_host("connected!\n");

	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	print_to_host("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	print_to_host("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	print_to_host("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));

	while(1) {
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void wifi_setup()
{
	ESP_ERROR_CHECK ( nvs_flash_init());
		tcpip_adapter_init();
		ESP_ERROR_CHECK ( esp_event_loop_init ( event_handler , NULL ) ) ;
		wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_ERROR_CHECK(esp_wifi_start());
		// configure the wifi connection and start the interface

		wifi_scan_config_t scan_config = {
				.ssid = 0,
				.bssid = 0,
				.channel = 0,
		        .show_hidden = true
		    };
		ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
		uint16_t ap_num = MAX_APs;
		wifi_ap_record_t ap_records [ MAX_APs ];
		ESP_ERROR_CHECK ( esp_wifi_scan_get_ap_records ( & ap_num , ap_records ) );

		wifi_config_t wifi_config = {
	        .sta = {
	            .ssid = WIFI_SSID,
	            .password = WIFI_PASS,
	        },
		};
		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));


		print_to_host("Found %d access points:\n", ap_num);
			print_to_host("\n");
			print_to_host("               SSID              | Channel | RSSI |   Auth Mode \n");
			print_to_host("----------------------------------------------------------------\n");
			for(int i = 0; i < ap_num; i++)
				print_to_host("%32s | %7d | %4d | %12s\n", (char*)ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi, getAuthModeName(ap_records[i].authmode));
			print_to_host("----------------------------------------------------------------\n");
		esp_apptrace_init();
		for(int i = 0; i < ap_num; i++)
		{
			print_to_host((char*)ap_records[0].ssid);
		}
}

void app_main()
{

	esp_log_set_vprintf(esp_apptrace_vprintf);

//	xTaskCreate(&wifi_task, "wifi_task", 2048, NULL, 5, NULL);
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(&get_data, "get_data", 2048, NULL, 1, NULL);
//    xTaskCreate(&get_data_acc, "get_data", 2048, NULL, 1, NULL);
}
