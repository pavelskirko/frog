/*
 * spi.c
 *
 *  Created on: Feb 2, 2019
 *      Author: Pavel
 */
#include "spi.h"


uint8_t who_am_i[2];
uint8_t ad_rec;

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
				        .clock_speed_hz=2*1000*1000,           //Clock out at 2 MHz
				        .command_bits=0,
						.address_bits=8,
						.mode=0,                                //SPI mode 0
				        .spics_io_num=PIN_NUM_CS0,               //CS pin
				        .queue_size=7,  	//We want to be able to queue 7 transactions at a time
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

	trans.addr=ICM20602_ACCEL_CONFIG;
	trans.tx_data[0]=(2<<3);
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=ICM20602_ACCEL_CONFIG2;
	trans.tx_data[0]=(1<<3); // Used to bypass DLPF (4kHz sample rate)
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));

	trans.addr=ICM20602_FIFO_EN;
	trans.tx_data[0]=(1<<3) | (1<<4); // enable write acc and gyro data
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));




}

void acc_who_i_am(spi_device_handle_t * spi, uint8_t i)
{
	spi_transaction_t trans;
	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr = (0x80 | ICM20602_WHO_AM_I);
	trans.length = 16;
	trans.rxlength=8;
	trans.rx_data[0] = 1;
	trans.flags = SPI_TRANS_USE_RXDATA;
	ESP_ERROR_CHECK(spi_device_queue_trans(*spi, &trans, portMAX_DELAY));
	spi_transaction_t * r_trans;
	ESP_ERROR_CHECK(spi_device_get_trans_result(*spi, &r_trans, portMAX_DELAY));
	who_am_i[i] = r_trans->rx_data[0];
}


void adc_setup(spi_device_handle_t * spi2)
{
	spi_device_interface_config_t devcfg2={
						        .clock_speed_hz=10*1000,           //Clock out at 10 KHz
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
//	FinalResult data = FinalResult_init_default;
	SpiEventGroup = xEventGroupCreate();
	Accel a1[NUM_OF_FIELDS];
	spi_device_handle_t spi1;
	spi_device_handle_t spi2;
	spi_device_handle_t spi3;
	spi_setup(&spi1, &spi2, &spi3);
	accel_init(&spi1);
	accel_init(&spi2);
//	vTaskDelay(1 / portTICK_PERIOD_MS);
	acc_who_i_am(&spi1, 0);  // test icm-20602: write to who_am_i global variable dec18
	acc_who_i_am(&spi2, 1);
	uint16_t num = 0;
	uint16_t data_x[NUM_OF_FIELDS];
	uint16_t data_y[NUM_OF_FIELDS];
	uint16_t data_z[NUM_OF_FIELDS];
	pb_ostream_t stream = pb_ostream_from_buffer(buff, sizeof(buff));
	while(num < NUM_OF_FIELDS)
	{
    	if((check_intr(&spi1) & 1) && num < NUM_OF_FIELDS)
    	{

    		a1[num].a_x = get_data_acc(&spi1, ICM20602_ACCEL_XOUT_L, ICM20602_ACCEL_XOUT_H);
    		a1[num].a_y = get_data_acc(&spi1, ICM20602_ACCEL_YOUT_L, ICM20602_ACCEL_YOUT_H);
    		a1[num].a_z = get_data_acc(&spi1, ICM20602_ACCEL_ZOUT_L, ICM20602_ACCEL_ZOUT_H);
    		indic(1);
    		num++;
    	}
    }
	xEventGroupSetBits(SpiEventGroup, BIT1);
	const TickType_t xTicksToWait = 100000 / portTICK_PERIOD_MS;
	for(uint16_t i = 0; i < NUM_OF_FIELDS; i++)
	{
		xEventGroupWaitBits(SpiEventGroup,    // The event group being tested.
		                 BIT1,  // The bits within the event group to wait for.
		                 pdTRUE,         // BIT_0 and BIT_4 should be cleared before returning.
		                 pdFALSE,        // Don't wait for both bits, either bit will do.
		                 xTicksToWait );
   		memset(buff, 0, BUFF_SIZE);
   		memset(&stream, 0, sizeof(pb_ostream_t));
   		stream = pb_ostream_from_buffer(buff, sizeof(buff));
	    if(pb_encode(&stream, Accel_fields, &a1[i]))
		{
	    	xEventGroupSetBits(SpiEventGroup, BIT0);
		}
	}

	while(1)
	{

	}
}

uint8_t check_intr(spi_device_handle_t * spi)
{
	spi_transaction_t trans;
	uint8_t intr;// = pvPortMallocCaps(8, MALLOC_CAP_DMA);

	memset(&trans, 0, sizeof(spi_transaction_t));
	trans.addr = (0x80 | ICM20602_INT_STATUS);
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
	uint8_t low_b;// = pvPortMallocCaps(8, MALLOC_CAP_DMA);
	uint8_t high_b;// = pvPortMallocCaps(8, MALLOC_CAP_DMA);

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
	uint16_t x = low_b | (high_b<<8);
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
