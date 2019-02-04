#include "tcp_server.h"

#define LISTENQ 2
#define MESSAGE "Hello TCP Client!!"
#define AP_SSID "ESP_32"
#define AP_PASSPHARSE "12345678"
#define AP_SSID_HIDDEN 0
#define AP_MAX_CONNECTIONS 4
#define AP_AUTHMODE WIFI_AUTH_OPEN // the passpharese should be atleast 8 chars long
#define AP_BEACON_INTERVAL 100 // in milli seconds


const int CLIENT_CONNECTED_BIT = BIT0;
const int CLIENT_DISCONNECTED_BIT = BIT1;
const int AP_STARTED_BIT = BIT2;
static const char *TAG = "tcp_server";

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
		printf("Event:ESP32 is started in AP mode\n");
        xEventGroupSetBits(wifi_event_group, AP_STARTED_BIT);
        break;

	case SYSTEM_EVENT_AP_STACONNECTED:
		xEventGroupSetBits(wifi_event_group, CLIENT_CONNECTED_BIT);
		break;

	case SYSTEM_EVENT_AP_STADISCONNECTED:
		xEventGroupSetBits(wifi_event_group, CLIENT_DISCONNECTED_BIT);
		break;
    default:
        break;
    }
    return ESP_OK;
}

void start_dhcp_server(){

    // initialize the tcp stack
	tcpip_adapter_init();
    // stop DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    // assign a static IP to the network interface
    tcpip_adapter_ip_info_t info;
    memset(&info, 0, sizeof(info));
    IP4_ADDR(&info.ip, 192, 168, 0, 1);
    IP4_ADDR(&info.gw, 192, 168, 0, 1);//ESP acts as router, so gw addr will be its own addr
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
    // start the DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
    printf("DHCP server started \n");
}
void initialise_wifi_in_ap(void)
{
    esp_log_level_set("wifi", ESP_LOG_NONE); // disable wifi driver logging
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );

    // configure the wifi connection and start the interface
	wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .password = AP_PASSPHARSE,
			.ssid_len = 0,
			.channel = 0,
			.authmode = AP_AUTHMODE,
			.ssid_hidden = AP_SSID_HIDDEN,
			.max_connection = AP_MAX_CONNECTIONS,
			.beacon_interval = AP_BEACON_INTERVAL,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK( esp_wifi_start() );
    printf("ESP WiFi started in AP mode \n");
}

void indic(int count)
{
    for(int i = 0; i < count; i++)
    {
    	gpio_set_level(26, 1);
    	vTaskDelay(50 / portTICK_PERIOD_MS);
    	gpio_set_level(26, 0);
    	vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void tcp_server(void *pvParameters)
{
    ESP_LOGI(TAG,"tcp_server task started \n");
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons( 3000 );
    int s, r;
    char recv_buf[64];
    char send_buf[64];
    static struct sockaddr_in remote_addr;
    static unsigned int socklen;
    socklen = sizeof(remote_addr);
    int cs;//client socket
    xEventGroupWaitBits(wifi_event_group,AP_STARTED_BIT,false,true,portMAX_DELAY);
    while(1){
        s = socket(AF_INET, SOCK_STREAM, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.\n");
//            indic(1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket\n");
         if(bind(s, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
            ESP_LOGE(TAG, "... socket bind failed errno=%d \n", errno);
            close(s);
//            indic(2);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket bind done \n");
        if(listen (s, LISTENQ) != 0) {
            ESP_LOGE(TAG, "... socket listen failed errno=%d \n", errno);
            close(s);
//            indic(3);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        while(1)
        {

            cs=accept(s,(struct sockaddr *)&remote_addr, &socklen);
            ESP_LOGI(TAG,"New connection request,Request data:");
            //set O_NONBLOCK so that recv will return, otherwise we need to impliment message end
            //detection logic. If know the client message format you should instead impliment logic
            //detect the end of message
            fcntl(cs,F_SETFL,O_NONBLOCK);
        	uint16_t r = 0;
            while(1)
            {
            	const TickType_t xTicksToWait = 1000000 / portTICK_PERIOD_MS;
            	xEventGroupWaitBits(
            	                 SpiEventGroup,    // The event group being tested.
            	                 BIT0 | BIT3,  // The bits within the event group to wait for.
            	                 pdTRUE,         // BIT_0 and BIT_4 should be cleared before returning.
            	                 pdFALSE,        // Don't wait for both bits, either bit will do.
            	                 xTicksToWait );
            	r = 0;
            	while(r < sizeof(buff1))
            	{
            		r = write(cs , buff1, sizeof(buff1));
            	}
            	r = 0;
            	while(r < sizeof(buff2))
            	{
            		r = write(cs , buff2, sizeof(buff2));
            	}
            	indic(1);
            	xEventGroupSetBits(SpiEventGroup, BIT1);
            	if(xEventGroupGetBits(SpiEventGroup) & BIT3)
            	{
            		Accel a = Accel_init_default;
            		a.last_msg = true;
            		memset(buff1, 0, BUFF_SIZE);
            		pb_ostream_t stream = pb_ostream_from_buffer(buff1, sizeof(buff1));
            		pb_encode(&stream, Accel_fields, &a);
            		vTaskDelay(100 / portTICK_PERIOD_MS);

            		write(cs , buff1, sizeof(buff1));
            	}

            }

//            {
//                ESP_LOGE(TAG, "... Send failed \n");
//                close(s);
//                vTaskDelay(4000 / portTICK_PERIOD_MS);
//                continue;
//            }
            ESP_LOGI(TAG, "... socket send success");
            indic(4);
            close(cs);
        }
        ESP_LOGI(TAG, "... server will be opened in 5 seconds");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "...tcp_client task closed\n");
}
