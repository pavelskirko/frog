/*
 * spi.c
 *
 *  Created on: Feb 2, 2019
 *      Author: Pavel
 */
#include "spi.h"


uint8_t who_am_i[2];
uint8_t ad_rec;
//Accel a3;
//size_t heap1;
//size_t heap2;
//size_t accel_size;

void spi_setup(spi_device_handle_t * spi1, spi_device_handle_t * spi2, spi_device_handle_t * spi3)
{
	spi_bus_config_t buscfg={
				        .miso_io_num=PIN_NUM_VSPI_Q,
				        .mosi_io_num=PIN_NUM_VSPI_D,
				        .sclk_io_num=PIN_NUM_CLK,
				        .quadwp_io_num=-1,
				        .quadhd_io_num=-1,
				        .max_transfer_sz=4094*16,
				    };
	spi_device_interface_config_t devcfg={
				        .clock_speed_hz=2*1000*1000,           //Clock out at 10 MHz
				        .command_bits=0,
						.address_bits=8,
						.mode=0,                                //SPI mode 0
				        .spics_io_num=PIN_NUM_CS0,               //CS pin
				        .queue_size=70,  	//We want to be able to queue 7 transactions at a time
						.flags=0,//SPI_DEVICE_POSITIVE_CS,
				    };
	ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi1));
	devcfg.spics_io_num = PIN_NUM_CS1;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi2));
	devcfg.spics_io_num = PIN_NUM_CS2;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, spi3));
}



void accel_init(spi_device_handle_t * spi)
{
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr=ICM20602_PWR_MGMT_1;
	trans.flags=SPI_TRANS_USE_TXDATA;
	trans.length=16;
	trans.tx_data[0]=(1<<5) | (1<<4) | 1; //cycle mode | gyro stanby | auto select clock
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=0x1A;
	trans.tx_data[0]=0; // should be for fifo
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));


	trans.addr=ICM20602_ACCEL_CONFIG;
	trans.tx_data[0]=(1<<3)| (1<<4);
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=ICM20602_ACCEL_CONFIG2;
	trans.tx_data[0]=(1<<3); // Used to bypass DLPF (4kHz sample rate)
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=ICM20602_USER_CTRL;
	trans.tx_data[0]=(1<<6); // fifo enable
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=ICM20602_FIFO_EN;
	trans.tx_data[0]=(1<<3) ; // enable write acc data
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=ICM20602_FIFO_WM_TH2;
	trans.tx_data[0]=0xFF; // treshold that trigers intr
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));





}

void acc_who_i_am(spi_device_handle_t * spi, uint8_t i)
{
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr = (0x80 | 0x1A);//ICM20602_WHO_AM_I);
	trans.length = 16;
	trans.rxlength=8;
	trans.rx_data[0] = 1;
	trans.flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	who_am_i[i] = r_trans->rx_data[0];

//	trans.addr = (0x80 | ICM20602_ACCEL_CONFIG);
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	memset(&r_trans, 0, sizeof(spi_transaction_t));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
//
//	trans.addr = (0x80 | 0x1A);
//	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
//	memset(&r_trans, 0, sizeof(spi_transaction_t));
//	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

//	who_am_i[i] = r_trans->rx_data[0];

}


void adc_setup(spi_device_handle_t * spi2)
{
	spi_device_interface_config_t devcfg2={
						        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
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
	SpiEventGroup = xEventGroupCreate();
	xTaskToNotify = NULL;
	Accel a1[4]; //= malloc(NUM_OF_FIELDS * sizeof(Accel));
	Accel a2[4];
	spi_device_handle_t spi1;
	spi_device_handle_t spi2;
	spi_device_handle_t spi3;
//	heap1 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
//	heap2 = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
//	accel_size = sizeof(Accel);
	spi_setup(&spi1, &spi2, &spi3);
	accel_init(&spi1);
	accel_init(&spi2);
//	acc_who_i_am(&spi1, 0);  // test icm-20602: write to who_am_i global variable dec18
//	acc_who_i_am(&spi2, 1);
	uint32_t num1 = 0;
	uint32_t num2 = 0;
	uint8_t buf[sizeof(Accel)*4];
	pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
	pb_istream_t stream_in = pb_istream_from_buffer(buf, sizeof(buf));
	partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
	esp_partition_erase_range(partition, 0, partition->size);

//	pb_ostream_t stream1 = pb_ostream_from_buffer(buff1, sizeof(buff1));
//	pb_ostream_t stream2 = pb_ostream_from_buffer(buff2, sizeof(buff2));
	xTaskToNotify = xTaskGetCurrentTaskHandle();

	uint64_t time;
	uint64_t pr_time1 = 0;
	uint64_t pr_time2 = 0;
//	ulTaskNotifyTake(pdTRUE,  portMAX_DELAY);
	int16_t * dma_buf = (int16_t *)heap_caps_malloc(256, MALLOC_CAP_DMA);
	timer_start(0, 0);
	while(num1 < NUM_OF_FIELDS || num2 < NUM_OF_FIELDS)
	{
    	if((check_intr(&spi1) & (1<<6)) && num1 < NUM_OF_FIELDS)
    	{
    		memset(a1, 0, sizeof(Accel));
    		memset(dma_buf, 0, 256);
    		get_data_acc_fifo(&spi1, dma_buf);
    		timer_get_counter_value(0,0, &time);
    		a1[0].time = (uint32_t) ((time - pr_time1) / 800); // to microsec*10
    		pr_time1 = time;
    		uint8_t count = 0;
    		for(uint8_t i = 0; i < 4; i++)
    		{
//    			memcpy(&dma_buf[count], &a1[i].a_x, sizeof(uint16_t));
//    			memcpy(&dma_buf[count+1], &a1[i].a_y, sizeof(uint16_t));
//    			memcpy(&dma_buf[count+2], &a1[i].a_z, sizeof(uint16_t));
    			a1[i].a_x = dma_buf[count];
    			a1[i].a_y = dma_buf[count+1];
    			a1[i].a_z = dma_buf[count+2];
    			a1[i].number = num1 + i;
    			count = count + 4;
    			a1[i].up = true;
    			a1[i].last_msg = false;
    			if(i > 0)
    			{
    				a1[i].time = 47;
    			}
//    			size_t data_size;
//    			pb_get_encoded_size(&data_size, Accel_fields, &a1[num1+i]);

    			memset(buf, 0, sizeof(buf));
    			memset(&stream, 0, sizeof(pb_ostream_t));
    			stream = pb_ostream_from_buffer(buf, sizeof(buf));
    			pb_encode(&stream, Accel_fields, &a1[i]);
    			esp_partition_write(partition, (sizeof(buf))*(num1+i), buf, sizeof(buf));
//    			memset(buf, 0, sizeof(buf));
//    			memset(&stream_in, 0, sizeof(pb_istream_t));
//    			stream_in = pb_istream_from_buffer(buf, sizeof(buf));
//    			esp_partition_read(partition, (sizeof(buf))*(num1+i), buf, sizeof(buf));
//
//    			pb_decode(&stream_in, Accel_fields, &a3);

    		}
    		num1 = num1+4;
    	}
    	if((check_intr(&spi2) & (1<<6)) && num1 < NUM_OF_FIELDS)
    	{
    		memset(a1, 0, sizeof(Accel));
   		    memset(dma_buf, 0, 256);
   		    get_data_acc_fifo(&spi2, dma_buf);
   		    timer_get_counter_value(0,0, &time);
    		a1[0].time = (uint32_t) ((time - pr_time1) / 800); // to microsec*10
    		pr_time1 = time;
    		uint8_t count = 0;
    		for(uint8_t i = 0; i < 4; i++)
    		{
    			a1[i].a_x = dma_buf[count];
    			a1[i].a_y = dma_buf[count+1];
    			a1[i].a_z = dma_buf[count+2];
    			a1[i].number = num1 + i;
    		    count = count + 4;
    		    a1[i].up = false;
    		    a1[i].last_msg = false;
    		    if(i > 0)
    		    {
    		    	a1[i].time = 0;
    		    }
    		    memset(buf, 0, sizeof(buf));
    		    memset(&stream, 0, sizeof(pb_ostream_t));
    		    stream = pb_ostream_from_buffer(buf, sizeof(buf));
    		    pb_encode(&stream, Accel_fields, &a1[i]);
    		    esp_partition_write(partition, (sizeof(buf))*(NUM_OF_FIELDS + num1+i), buf, sizeof(buf));
    		}
    		num2 = num2 +4;
    	}

    }
	xEventGroupSetBits(SpiEventGroup, BIT1);
	indic(3);
//	while(1)
//	{
//		for(uint16_t i = 0; i < NUM_OF_FIELDS; i++)
//		{
//			xEventGroupWaitBits(SpiEventGroup,    // The event group being tested.
//					BIT1,  // The bits within the event group to wait for.
//					pdTRUE,         // should be cleared before returning.
//					pdFALSE,        // Don't wait for both bits, either bit will do.
//					portMAX_DELAY );
//			memset(buff1, 0, BUFF_SIZE);
////   		memset(buff2, 0, BUFF_SIZE);
//			memset(&stream1, 0, sizeof(pb_ostream_t));
//   		memset(&stream2, 0, sizeof(pb_ostream_t));
//			stream1 = pb_ostream_from_buffer(buff1, BUFF_SIZE);
//   		stream2 = pb_ostream_from_buffer(buff2, BUFF_SIZE);
//			pb_encode(&stream1, Accel_fields, &a1[i]);
//   		pb_encode(&stream2, Accel_fields, &a2[i]);
//	    	if(pb_encode(&stream1, Accel_fields, &a1[i]) && pb_encode(&stream2, Accel_fields, &a2[i]))
//			{
//			vTaskDelay(10 / portTICK_PERIOD_MS);
//			xEventGroupSetBits(SpiEventGroup, BIT0);
//			}
//		}
//		xEventGroupSetBits(SpiEventGroup, BIT3);
//	}


	while(1)
	{

	}
}

void get_data_acc_fifo(spi_device_handle_t * spi, int16_t * dma_buf)
{

	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr = (0x80 | ICM20602_FIFO_R_W);
	trans.rx_buffer=dma_buf;
	trans.length = 256+8;
	trans.rxlength=256;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans1;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
}

uint8_t check_intr(spi_device_handle_t * spi)
{
	spi_transaction_t trans;
	uint8_t intr;// = pvPortMallocCaps(8, MALLOC_CAP_DMA);

	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr = (0x80 | ICM20602_FIFO_WM_INT);
	trans.rx_buffer=&intr;
	trans.length = 16;
	trans.rxlength=8;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans1;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
	return(intr);
}

uint16_t get_data_acc(spi_device_handle_t * spi, uint8_t addr_low, uint8_t addr_high)
{
	spi_transaction_t trans;
	uint32_t low_b;//= heap_caps_malloc(32, MALLOC_CAP_DMA);
	uint32_t high_b;//= heap_caps_malloc(32, MALLOC_CAP_DMA);

	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr = (0x80 | addr_low);
	trans.rx_buffer=&low_b;
	trans.length = 16;
	trans.rxlength=8;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans1;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));

	trans.addr = (0x80 | addr_high);
	trans.rx_buffer=&high_b;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans1, portMAX_DELAY));
//	uint8_t h = (uint8_t)*high_b;
//	uint8_t l = (uint8_t)*low_b;
	uint16_t x = low_b | (high_b<<8);
//	heap_caps_free(low_b);
//	heap_caps_free(high_b);
	return(x);
}

void get_data_adc(void *pvParameter)
{
	spi_device_handle_t spi2;
//	spi_bus_config_t buscfg={
//				        .miso_io_num=PIN_NUM_VSPI_Q,
//				        .mosi_io_num=PIN_NUM_VSPI_D,
//				        .sclk_io_num=PIN_NUM_CLK,
//				        .quadwp_io_num=-1,
//				        .quadhd_io_num=-1,
//				        .max_transfer_sz=4094,
//				    };
//	spi_device_interface_config_t devcfg={
//				        .clock_speed_hz=10*1000,           //Clock out at 10 MHz
//				        .command_bits=8,
//						.address_bits=8,
//						.mode=0,                                //SPI mode 0
//				        .spics_io_num=PIN_NUM_CS2,               //CS pin
//				        .queue_size=7,  	//We want to be able to queue 7 transactions at a time
//						.flags=0,//SPI_DEVICE_POSITIVE_CS,
//				    };
//	ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));
//	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi2));
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
