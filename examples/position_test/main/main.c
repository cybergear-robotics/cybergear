#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/twai.h"

#include "cybergear.h"
#include "cybergear_utils.h"

#define TAG "position_test"

#define TWAI_ALERTS ( TWAI_ALERT_RX_DATA | \
		TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | \
		TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | \
		TWAI_ALERT_BUS_ERROR )

#define POLLING_RATE_TICKS pdMS_TO_TICKS(500)

void app_main(void)
{
    /* initialize configuration structures using macro initializers */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
		(gpio_num_t) CONFIG_CYBERGEAR_CAN_TX, 
		(gpio_num_t) CONFIG_CYBERGEAR_CAN_RX, 
		TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* install TWAI driver */
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERTS, NULL));
	
	/* initialize cybergear motor */
	cybergear_motor_t cybergear_motor;
	cybergear_init(&cybergear_motor, CONFIG_CYBERGEAR_MASTER_CAN_ID, CONFIG_CYBERGEAR_MOTOR_CAN_ID, POLLING_RATE_TICKS);
	ESP_ERROR_CHECK(cybergear_stop(&cybergear_motor));
	cybergear_set_mode(&cybergear_motor, CYBERGEAR_MODE_POSITION);
	cybergear_set_limit_speed(&cybergear_motor, 3.0f);
	cybergear_set_limit_current(&cybergear_motor, 5.0f);
	cybergear_enable(&cybergear_motor);
	cybergear_set_position(&cybergear_motor, 10.0); 

	uint32_t alerts_triggered;
	twai_status_info_t twai_status;
	twai_message_t message;
	cybergear_status_t status;
	while(1)
	{
		/* request status */
		cybergear_request_status(&cybergear_motor);

		/* handle CAN alerts */ 
		twai_read_alerts(&alerts_triggered, POLLING_RATE_TICKS);
		twai_get_status_info(&twai_status);
		if (alerts_triggered & TWAI_ALERT_ERR_PASS)
		{
			ESP_LOGE(TAG, "Alert: TWAI controller has become error passive.");
		}
		if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
		{
			ESP_LOGE(TAG, "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
			ESP_LOGE(TAG, "Bus error count: %lu\n", twai_status.bus_error_count);
		}
		if (alerts_triggered & TWAI_ALERT_TX_FAILED)
		{
			ESP_LOGE(TAG, "Alert: The Transmission failed.");
			ESP_LOGE(TAG, "TX buffered: %lu\t", twai_status.msgs_to_tx);
			ESP_LOGE(TAG, "TX error: %lu\t", twai_status.tx_error_counter);
			ESP_LOGE(TAG, "TX failed: %lu\n", twai_status.tx_failed_count);
		}
		/* handle received messages */
		if (alerts_triggered & TWAI_ALERT_RX_DATA) 
		{
			while (twai_receive(&message, 0) == ESP_OK)
			{
				cybergear_process_message(&cybergear_motor, &message);
			}
			/* get cybergear status*/
			cybergear_get_status(&cybergear_motor, &status);
			cybergear_print_status(&status);
			/* get cybergear faults */
			if(cybergear_has_faults(&cybergear_motor))
			{
				cybergear_fault_t faults;
				cybergear_get_faults(&cybergear_motor, &faults);
				cybergear_print_faults(&faults);
			}
		}
	}
}
