#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "unity.h"
#include "unity_test_runner.h"

#include "cybergear.h"

#define RX_QUEUE_LEN 32
#define FRAME_TIMEOUT_MS 100
#define RESPONSE_TIMEOUT_MS 300

static const char *TAG = "cybergear_test";

typedef struct {
    uint32_t identifier;
    uint8_t data[8];
} rx_frame_t;

typedef struct {
    QueueHandle_t rx_queue;
    uint32_t feedback_count;
    uint32_t parameter_count;
    uint32_t device_info_count;
    uint16_t last_parameter_address;
} test_context_t;

static esp_err_t twai_send(void *context, uint32_t identifier, const uint8_t *data, size_t data_length)
{
    twai_node_handle_t node = context;
    twai_frame_t frame = {
        .header.id = identifier,
        .header.ide = true,
        .buffer = (uint8_t *)data,
        .buffer_len = data_length,
    };
    esp_err_t err = twai_node_transmit(node, &frame, FRAME_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    return twai_node_transmit_wait_all_done(node, FRAME_TIMEOUT_MS);
}

static bool twai_rx_done_cb(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    test_context_t *context = user_ctx;
    rx_frame_t rx_frame;
    twai_frame_t frame = {
        .buffer = rx_frame.data,
        .buffer_len = sizeof(rx_frame.data),
    };
    BaseType_t task_woken = pdFALSE;
    (void)edata;
    if (twai_node_receive_from_isr(node, &frame) == ESP_OK && frame.header.ide && frame.header.dlc == sizeof(rx_frame.data)) {
        rx_frame.identifier = frame.header.id;
        (void)xQueueSendFromISR(context->rx_queue, &rx_frame, &task_woken);
    }
    return task_woken == pdTRUE;
}

static void process_received_frames(cybergear_motor_t *motor, test_context_t *context)
{
    rx_frame_t frame;
    while (xQueueReceive(context->rx_queue, &frame, 0) == pdTRUE) {
        cybergear_message_t message = {
            .identifier = frame.identifier,
            .data = frame.data,
            .data_length = sizeof(frame.data),
        };
        uint8_t packet_type = (frame.identifier >> 24) & 0x3f;
        esp_err_t err = cybergear_process_message(motor, &message);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Ignored frame type %u: %s", packet_type, esp_err_to_name(err));
            continue;
        }
        if (packet_type == CMD_REQUEST) {
            context->feedback_count++;
        } else if (packet_type == CMD_GET_DEVICE_ID) {
            context->device_info_count++;
        } else if (packet_type == CMD_RAM_READ) {
            context->parameter_count++;
            context->last_parameter_address = frame.data[0] | frame.data[1] << 8;
        }
    }
}

static bool wait_for_count(cybergear_motor_t *motor, test_context_t *context, uint32_t *count, uint32_t previous, uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    do {
        process_received_frames(motor, context);
        if (*count > previous) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (xTaskGetTickCount() < deadline);
    process_received_frames(motor, context);
    return *count > previous;
}

static bool expect_ok(const char *name, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PASS command %s", name);
        return true;
    }
    ESP_LOGE(TAG, "FAIL command %s: %s", name, esp_err_to_name(err));
    return false;
}

static int scan_motor_id(twai_node_handle_t node, test_context_t *context, uint8_t master_can_id)
{
    uint8_t data[8] = { 0 };
    for (uint16_t candidate = 0; candidate < 128; candidate++) {
        uint32_t identifier = CMD_GET_STATUS << 24 | master_can_id << 8 | candidate;
        if (twai_send(node, identifier, data, sizeof(data)) != ESP_OK) {
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        rx_frame_t frame;
        while (xQueueReceive(context->rx_queue, &frame, 0) == pdTRUE) {
            uint8_t packet_type = (frame.identifier >> 24) & 0x3f;
            uint8_t motor_can_id = (frame.identifier >> 8) & 0xff;
            if (packet_type == CMD_REQUEST) {
                ESP_LOGI(TAG, "Discovered motor CAN ID %u", motor_can_id);
                return motor_can_id;
            }
        }
    }
    return -1;
}

static bool request_status(cybergear_motor_t *motor, test_context_t *context, cybergear_status_t *status, const char *name)
{
    uint32_t previous = context->feedback_count;
    if (!expect_ok(name, cybergear_request_status(motor))) {
        return false;
    }
    if (!wait_for_count(motor, context, &context->feedback_count, previous, RESPONSE_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "FAIL %s: no feedback within %d ms", name, RESPONSE_TIMEOUT_MS);
        return false;
    }
    if (!expect_ok("cybergear_get_status", cybergear_get_status(motor, status))) {
        return false;
    }
    ESP_LOGI(TAG, "%s: state=%d pos=%.4f speed=%.4f torque=%.4f temp=%.1f", name,
             status->state, status->position, status->speed, status->torque, status->temperature);
    return true;
}

static bool read_parameter(cybergear_motor_t *motor, test_context_t *context, uint16_t address, const char *name)
{
    uint32_t previous = context->parameter_count;
    if (!expect_ok(name, cybergear_get_param(motor, address))) {
        return false;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS);
    do {
        process_received_frames(motor, context);
        if (context->parameter_count > previous && context->last_parameter_address == address) {
            ESP_LOGI(TAG, "PASS read %s (0x%04x)", name, address);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (xTaskGetTickCount() < deadline);
    process_received_frames(motor, context);
    if (context->parameter_count > previous && context->last_parameter_address == address) {
        ESP_LOGI(TAG, "PASS read %s (0x%04x)", name, address);
        return true;
    }
    ESP_LOGE(TAG, "FAIL read %s (0x%04x): no matching response", name, address);
    return false;
}

static test_context_t context;
static cybergear_motor_t motor;
static cybergear_config_t config = {
    .send = twai_send,
    .mode = CYBERGEAR_MODE_POSITION,
    .master_can_id = CONFIG_CYBERGEAR_MASTER_CAN_ID,
    .can_id = CONFIG_CYBERGEAR_MOTOR_CAN_ID,
    .speed_limit = 1.0f,
    .current_limit = 1.0f,
    .torque_limit = 1.0f,
    .enable_on_init = true,
};
static twai_node_handle_t node;

static void prepare_motor(cybergear_status_t *status)
{
    int discovered_can_id = scan_motor_id(node, &context, config.master_can_id);
    TEST_ASSERT_GREATER_OR_EQUAL(0, discovered_can_id);
    config.can_id = discovered_can_id;
    memset(&motor, 0, sizeof(motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_init(&motor, &config));
    TEST_ASSERT_TRUE(request_status(&motor, &context, status, "baseline_status"));
    if (status->state != CYBERGEAR_STATE_RUNNING) {
        TEST_ASSERT_EQUAL(ESP_OK, cybergear_enable(&motor));
        TEST_ASSERT_TRUE(request_status(&motor, &context, status, "enable_status"));
    }
    TEST_ASSERT_EQUAL(CYBERGEAR_STATE_RUNNING, status->state);
}

static void stop_motor(void)
{
    cybergear_status_t status;
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop(&motor));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "stop_status"));
    TEST_ASSERT_EQUAL(CYBERGEAR_STATE_RESET, status.state);
}

static void read_parameter_test(uint16_t address, const char *name)
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_TRUE(read_parameter(&motor, &context, address, name));
    stop_motor();
}

#define PARAMETER_TEST(name, address) \
    TEST_CASE("reads " name, "[hardware][parameter]") { read_parameter_test(address, name); }

PARAMETER_TEST("run mode", ADDR_RUN_MODE)
PARAMETER_TEST("Iq reference", ADDR_IQ_REF)
PARAMETER_TEST("speed reference", ADDR_SPEED_REF)
PARAMETER_TEST("torque limit", ADDR_LIMIT_TORQUE)
PARAMETER_TEST("current Kp", ADDR_CURRENT_KP)
PARAMETER_TEST("current Ki", ADDR_CURRENT_KI)
PARAMETER_TEST("current filter gain", ADDR_CURRENT_FILTER_GAIN)
PARAMETER_TEST("position reference", ADDR_LOC_REF)
PARAMETER_TEST("speed limit", ADDR_LIMIT_SPEED)
PARAMETER_TEST("current limit", ADDR_LIMIT_CURRENT)
PARAMETER_TEST("mechanical position", ADDR_MECH_POS)
PARAMETER_TEST("filtered Iq", ADDR_IQF)
PARAMETER_TEST("mechanical velocity", ADDR_MECH_VEL)
PARAMETER_TEST("bus voltage", ADDR_VBUS)
PARAMETER_TEST("rotation count", ADDR_ROTATION)
PARAMETER_TEST("position Kp", ADDR_LOC_KP)
PARAMETER_TEST("speed Kp", ADDR_SPD_KP)
PARAMETER_TEST("speed Ki", ADDR_SPD_KI)

TEST_CASE("stops and reports no faults", "[hardware][status]")
{
    cybergear_status_t status;
    cybergear_fault_t faults;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_get_faults(&motor, &faults));
    TEST_ASSERT_FALSE(cybergear_has_faults(&motor));
    stop_motor();
}

TEST_CASE("stops and clears faults explicitly", "[hardware][fault]")
{
    cybergear_status_t status;
    cybergear_fault_t faults;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop_and_clear_faults(&motor));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "clear_faults_status"));
    TEST_ASSERT_EQUAL(CYBERGEAR_STATE_RESET, status.state);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_get_faults(&motor, &faults));
    TEST_ASSERT_FALSE(cybergear_has_faults(&motor));
}

TEST_CASE("keeps type-21 warnings separate from faults", "[fault][warning]")
{
    cybergear_motor_t test_motor = { 0 };
    cybergear_config_t test_config = { .can_id = 1 };
    uint8_t data[8] = { 0 };
    cybergear_message_t message = {
        .identifier = CMD_GET_MOTOR_FAIL << 24 | 1 << 8,
        .data = data,
        .data_length = sizeof(data),
    };
    cybergear_fault_t faults;
    cybergear_warning_t warnings;

    test_motor.config = &test_config;
    data[4] = 0x01;
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_process_message(&test_motor, &message));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_get_faults(&test_motor, &faults));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_get_warnings(&test_motor, &warnings));
    TEST_ASSERT_FALSE(cybergear_has_faults(&test_motor));
    TEST_ASSERT_TRUE(cybergear_has_warnings(&test_motor));
    TEST_ASSERT_FALSE(faults.over_temperature);
    TEST_ASSERT_TRUE(warnings.over_temperature);
}

TEST_CASE("writes and reads back speed limit", "[hardware][write]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_limit_speed(&motor, 1.0f));
    TEST_ASSERT_TRUE(read_parameter(&motor, &context, ADDR_LIMIT_SPEED, "limit_speed"));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, motor.params.limit_spd);
    stop_motor();
}

TEST_CASE("writes and reads back current limit", "[hardware][write]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_limit_current(&motor, 1.0f));
    TEST_ASSERT_TRUE(read_parameter(&motor, &context, ADDR_LIMIT_CURRENT, "limit_current"));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, motor.params.limit_cur);
    stop_motor();
}

TEST_CASE("writes and reads back torque limit", "[hardware][write]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_limit_torque(&motor, 1.0f));
    TEST_ASSERT_TRUE(read_parameter(&motor, &context, ADDR_LIMIT_TORQUE, "limit_torque"));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, motor.params.limit_torque);
    stop_motor();
}

#define GAIN_WRITE_TEST(label, setter, value, address) \
    TEST_CASE("writes " label, "[hardware][write]") { \
        cybergear_status_t status; \
        prepare_motor(&status); \
        TEST_ASSERT_EQUAL(ESP_OK, setter(&motor, value)); \
        TEST_ASSERT_TRUE(read_parameter(&motor, &context, address, label)); \
        stop_motor(); \
    }

GAIN_WRITE_TEST("current Kp", cybergear_set_current_kp, 0.125f, ADDR_CURRENT_KP)
GAIN_WRITE_TEST("current Ki", cybergear_set_current_ki, 0.0158f, ADDR_CURRENT_KI)
GAIN_WRITE_TEST("current filter gain", cybergear_set_current_filter_gain, 0.1f, ADDR_CURRENT_FILTER_GAIN)
GAIN_WRITE_TEST("position Kp", cybergear_set_position_kp, 1.0f, ADDR_LOC_KP)
GAIN_WRITE_TEST("speed Kp", cybergear_set_speed_kp, 1.0f, ADDR_SPD_KP)
GAIN_WRITE_TEST("speed Ki", cybergear_set_speed_ki, 0.002f, ADDR_SPD_KI)

TEST_CASE("runs motion mode at the current position", "[hardware][mode]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_mode(&motor, CYBERGEAR_MODE_MOTION));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_enable(&motor));
    cybergear_motion_cmd_t command = { .position = status.position, .speed = 0, .torque = 0, .kp = 0, .kd = 0 };
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_motion_cmd(&motor, &command));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "motion_status"));
    TEST_ASSERT_EQUAL(CYBERGEAR_STATE_RUNNING, status.state);
    stop_motor();
}

TEST_CASE("position mode enforces configured speed limit", "[hardware][mode][limit]")
{
    cybergear_status_t status;
    float initial_position;
    float maximum_speed = 0;
    prepare_motor(&status);
    initial_position = status.position;
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_mode(&motor, CYBERGEAR_MODE_POSITION));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_limit_speed(&motor, 1.0f));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_enable(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_position(&motor, initial_position + 1.5f));
    for (int i = 0; i < 50; i++) {
        TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "position_limit_status"));
        float speed = status.speed < 0 ? -status.speed : status.speed;
        if (speed > maximum_speed) maximum_speed = speed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(0.2f, maximum_speed);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(1.15f, maximum_speed);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_position(&motor, initial_position));
    vTaskDelay(pdMS_TO_TICKS(1800));
    stop_motor();
}

TEST_CASE("speed mode follows positive and negative commands", "[hardware][mode]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_mode(&motor, CYBERGEAR_MODE_SPEED));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_limit_current(&motor, 1.0f));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_enable(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_speed(&motor, 0.5f));
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "positive_speed_status"));
    TEST_ASSERT_GREATER_THAN_FLOAT(0.2f, status.speed);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_speed(&motor, -0.5f));
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "negative_speed_status"));
    TEST_ASSERT_LESS_THAN_FLOAT(-0.2f, status.speed);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_speed(&motor, 0));
    stop_motor();
}

TEST_CASE("current mode accepts a low current command", "[hardware][mode]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_mode(&motor, CYBERGEAR_MODE_CURRENT));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_enable(&motor));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_current(&motor, 0.1f));
    vTaskDelay(pdMS_TO_TICKS(200));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "current_status"));
    TEST_ASSERT_EQUAL(CYBERGEAR_STATE_RUNNING, status.state);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_current(&motor, 0));
    stop_motor();
}

TEST_CASE("keeps the current CAN ID", "[hardware][can]")
{
    cybergear_status_t status;
    cybergear_device_info_t device_info;
    prepare_motor(&status);
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_stop(&motor));
    uint32_t previous = context.device_info_count;
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_motor_can_id(&motor, config.can_id));
    TEST_ASSERT_TRUE(wait_for_count(&motor, &context, &context.device_info_count, previous, RESPONSE_TIMEOUT_MS));
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_get_device_info(&motor, &device_info));
    TEST_ASSERT_TRUE(device_info.updated);
    TEST_ASSERT_EQUAL(config.can_id, device_info.motor_can_id);
    TEST_ASSERT_EQUAL(0xfe, device_info.recipient_can_id);
    TEST_ASSERT_FALSE(device_info.unique_id[0] == 0 && device_info.unique_id[1] == 0 &&
                      device_info.unique_id[2] == 0 && device_info.unique_id[3] == 0 &&
                      device_info.unique_id[4] == 0 && device_info.unique_id[5] == 0 &&
                      device_info.unique_id[6] == 0 && device_info.unique_id[7] == 0);
    ESP_LOGI(TAG, "Device UID: %02x%02x%02x%02x%02x%02x%02x%02x",
             device_info.unique_id[0], device_info.unique_id[1], device_info.unique_id[2], device_info.unique_id[3],
             device_info.unique_id[4], device_info.unique_id[5], device_info.unique_id[6], device_info.unique_id[7]);
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "same_id_status"));
    stop_motor();
}

TEST_CASE("sets the stopped mechanical position to zero", "[hardware][zero]")
{
    cybergear_status_t status;
    prepare_motor(&status);
    stop_motor();
    TEST_ASSERT_EQUAL(ESP_OK, cybergear_set_mech_position_to_zero(&motor));
    TEST_ASSERT_TRUE(request_status(&motor, &context, &status, "mechanical_zero_status"));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0, status.position);
    stop_motor();
}

void app_main(void)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = (gpio_num_t)CONFIG_CYBERGEAR_CAN_TX,
        .io_cfg.rx = (gpio_num_t)CONFIG_CYBERGEAR_CAN_RX,
        .io_cfg.quanta_clk_out = -1,
        .io_cfg.bus_off_indicator = -1,
        .bit_timing.bitrate = 1000000,
        .tx_queue_depth = 10,
        .fail_retry_cnt = -1,
    };
    twai_event_callbacks_t callbacks = { .on_rx_done = twai_rx_done_cb };

    context.rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_frame_t));
    ESP_ERROR_CHECK(context.rx_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node));
    config.send_context = node;
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &callbacks, &context));
    ESP_ERROR_CHECK(twai_node_enable(node));

    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
    (void)cybergear_stop(&motor);
}
