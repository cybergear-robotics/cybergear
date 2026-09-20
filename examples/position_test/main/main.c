#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_system.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "cybergear.h"
#include "cybergear_utils.h"

#define TWAI_TIMEOUT_MS 100
#define POLLING_RATE_MS 100

static esp_err_t cybergear_twai_send(void *context, uint32_t identifier, const uint8_t *data, size_t data_length)
{
	twai_node_handle_t node = context;
	twai_frame_t frame = {
		.header.id = identifier,
		.header.ide = true,
		.buffer = (uint8_t *)data,
		.buffer_len = data_length,
	};
	esp_err_t err = twai_node_transmit(node, &frame, TWAI_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}
	/* The CyberGear buffer is owned by the caller, so finish before returning. */
	return twai_node_transmit_wait_all_done(node, TWAI_TIMEOUT_MS);
}

static bool twai_rx_done_cb(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	uint8_t data[8];
	twai_frame_t frame = {
		.buffer = data,
		.buffer_len = sizeof(data),
	};
	cybergear_motor_t *motor = user_ctx;

	(void)edata;
	if (twai_node_receive_from_isr(node, &frame) == ESP_OK && frame.header.dlc == sizeof(data)) {
		cybergear_message_t message = {
			.identifier = frame.header.id,
			.data = data,
			.data_length = frame.header.dlc,
		};
		cybergear_process_message(motor, &message);
	}
	return false;
}

void app_main(void)
{
	/* initialize cybergear motor */
	cybergear_motor_t cybergear_motor;
	cybergear_config_t cybergear_config = {
		.send = cybergear_twai_send,
		.mode = CYBERGEAR_MODE_POSITION,
		.master_can_id = CONFIG_CYBERGEAR_MASTER_CAN_ID,
		.can_id = CONFIG_CYBERGEAR_MOTOR_CAN_ID,
		.speed_limit = 3.0f,
		.current_limit = 5.0f,
		.torque_limit = 10.0f,
		.enable_on_init = true,
	};
	twai_onchip_node_config_t node_config = {
		.io_cfg.tx = (gpio_num_t)CONFIG_CYBERGEAR_CAN_TX,
		.io_cfg.rx = (gpio_num_t)CONFIG_CYBERGEAR_CAN_RX,
		.io_cfg.quanta_clk_out = -1,
		.io_cfg.bus_off_indicator = -1,
		.bit_timing.bitrate = 1000000,
		.tx_queue_depth = 5,
		.fail_retry_cnt = -1,
	};
	twai_event_callbacks_t callbacks = {
		.on_rx_done = twai_rx_done_cb,
	};
	twai_node_handle_t node;

	ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node));
	cybergear_config.send_context = node;
	ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &callbacks, &cybergear_motor));
	ESP_ERROR_CHECK(twai_node_enable(node));
	ESP_ERROR_CHECK(cybergear_init(&cybergear_motor, &cybergear_config));
	ESP_ERROR_CHECK(cybergear_set_position(&cybergear_motor, 10.0f));

	cybergear_status_t status;
	while(1)
	{
		/* request status */
		cybergear_request_status(&cybergear_motor);
		vTaskDelay(pdMS_TO_TICKS(POLLING_RATE_MS));
		/* Received frames are processed by twai_rx_done_cb. */
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
