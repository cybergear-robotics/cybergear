#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cybergear.h"
#include "cybergear_utils.h"
#include "twai_transport.h"

#define POLLING_RATE_MS 100

static bool cybergear_rx_listener(const twai_frame_t *frame, void *context)
{
    cybergear_motor_t *motor = context;
    if (!frame->header.ide || frame->header.dlc != 8) {
        return false;
    }

    const cybergear_message_t message = {
        .identifier = frame->header.id,
        .data = frame->buffer,
        .data_length = frame->header.dlc,
    };
    return cybergear_process_message(motor, &message) == ESP_OK;
}

/* Example listener for a standard-ID device sharing the same bus. */
static bool other_can_device_listener(const twai_frame_t *frame, void *context)
{
    volatile uint32_t *received_frames = context;
    if (!frame->header.ide && frame->header.id == 0x123) {
        (*received_frames)++;
        return true;
    }
    return false;
}

void app_main(void)
{
    cybergear_motor_t cybergear_motor;
    cybergear_config_t cybergear_config = {
        .send = twai_transport_send,
        .mode = CYBERGEAR_MODE_POSITION,
        .master_can_id = CONFIG_CYBERGEAR_MASTER_CAN_ID,
        .can_id = CONFIG_CYBERGEAR_MOTOR_CAN_ID,
        .speed_limit = 3.0f,
        .current_limit = 5.0f,
        .torque_limit = 10.0f,
        .enable_on_init = true,
    };
    const twai_onchip_node_config_t node_config = {
        .io_cfg.tx = (gpio_num_t)CONFIG_CYBERGEAR_CAN_TX,
        .io_cfg.rx = (gpio_num_t)CONFIG_CYBERGEAR_CAN_RX,
        .io_cfg.quanta_clk_out = -1,
        .io_cfg.bus_off_indicator = -1,
        .bit_timing.bitrate = 1000000,
        .tx_queue_depth = TWAI_TRANSPORT_TX_POOL_SIZE,
        .fail_retry_cnt = -1,
    };
    twai_transport_t transport;
    volatile uint32_t other_can_device_frames = 0;

    ESP_ERROR_CHECK(twai_transport_init(&transport, &node_config));
    ESP_ERROR_CHECK(twai_transport_add_listener(&transport, cybergear_rx_listener, &cybergear_motor));
    ESP_ERROR_CHECK(twai_transport_add_listener(&transport, other_can_device_listener, (void *)&other_can_device_frames));
    ESP_ERROR_CHECK(twai_transport_start(&transport));

    cybergear_config.send_context = &transport;
    ESP_ERROR_CHECK(cybergear_init(&cybergear_motor, &cybergear_config));
    ESP_ERROR_CHECK(cybergear_set_position(&cybergear_motor, 10.0f));

    while (true) {
        cybergear_status_t status;
        cybergear_request_status(&cybergear_motor);
        vTaskDelay(pdMS_TO_TICKS(POLLING_RATE_MS));
        cybergear_get_status(&cybergear_motor, &status);
        cybergear_print_status(&status);
    }
}
