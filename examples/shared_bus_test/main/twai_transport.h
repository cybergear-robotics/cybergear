#ifndef TWAI_TRANSPORT_H
#define TWAI_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"

#define TWAI_TRANSPORT_TX_POOL_SIZE 8
#define TWAI_TRANSPORT_MAX_LISTENERS 4

/* Listeners run in the TWAI ISR and must not retain the frame buffer. */
typedef bool (*twai_transport_rx_listener_t)(const twai_frame_t *frame, void *context);

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
    bool in_use;
} twai_transport_tx_slot_t;

typedef struct {
    twai_transport_rx_listener_t callback;
    void *context;
} twai_transport_listener_t;

typedef struct {
    twai_node_handle_t node;
    portMUX_TYPE lock;
    twai_transport_tx_slot_t tx_slots[TWAI_TRANSPORT_TX_POOL_SIZE];
    twai_transport_listener_t listeners[TWAI_TRANSPORT_MAX_LISTENERS];
    size_t listener_count;
} twai_transport_t;

esp_err_t twai_transport_init(twai_transport_t *transport, const twai_onchip_node_config_t *node_config);
/* Register listeners before calling twai_transport_start(). */
esp_err_t twai_transport_add_listener(twai_transport_t *transport, twai_transport_rx_listener_t callback, void *context);
esp_err_t twai_transport_start(twai_transport_t *transport);

/* Compatible with cybergear_send_fn_t. */
esp_err_t twai_transport_send(void *context, uint32_t identifier, const uint8_t *data, size_t data_length);

#endif
