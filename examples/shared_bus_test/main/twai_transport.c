#include <string.h>

#include "twai_transport.h"

static bool twai_transport_rx_done_cb(twai_node_handle_t node, const twai_rx_done_event_data_t *event, void *user_context)
{
    twai_transport_t *transport = user_context;
    uint8_t data[TWAI_FRAME_MAX_LEN];
    twai_frame_t frame = {
        .buffer = data,
        .buffer_len = sizeof(data),
    };

    (void)event;
    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) {
        return false;
    }

    for (size_t i = 0; i < transport->listener_count; i++) {
        if (transport->listeners[i].callback(&frame, transport->listeners[i].context)) {
            break;
        }
    }
    return false;
}

static bool twai_transport_tx_done_cb(twai_node_handle_t node, const twai_tx_done_event_data_t *event, void *user_context)
{
    twai_transport_t *transport = user_context;

    (void)node;
    portENTER_CRITICAL_ISR(&transport->lock);
    for (size_t i = 0; i < TWAI_TRANSPORT_TX_POOL_SIZE; i++) {
        if (&transport->tx_slots[i].frame == event->done_tx_frame) {
            transport->tx_slots[i].in_use = false;
            break;
        }
    }
    portEXIT_CRITICAL_ISR(&transport->lock);
    return false;
}

esp_err_t twai_transport_init(twai_transport_t *transport, const twai_onchip_node_config_t *node_config)
{
    if (transport == NULL || node_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(transport, 0, sizeof(*transport));
    portMUX_INITIALIZE(&transport->lock);
    return twai_new_node_onchip(node_config, &transport->node);
}

esp_err_t twai_transport_add_listener(twai_transport_t *transport, twai_transport_rx_listener_t callback, void *context)
{
    if (transport == NULL || callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (transport->listener_count == TWAI_TRANSPORT_MAX_LISTENERS) {
        return ESP_ERR_NO_MEM;
    }

    transport->listeners[transport->listener_count++] = (twai_transport_listener_t) {
        .callback = callback,
        .context = context,
    };
    return ESP_OK;
}

esp_err_t twai_transport_start(twai_transport_t *transport)
{
    if (transport == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const twai_event_callbacks_t callbacks = {
        .on_rx_done = twai_transport_rx_done_cb,
        .on_tx_done = twai_transport_tx_done_cb,
    };
    esp_err_t err = twai_node_register_event_callbacks(transport->node, &callbacks, transport);
    if (err != ESP_OK) {
        return err;
    }
    return twai_node_enable(transport->node);
}

esp_err_t twai_transport_send(void *context, uint32_t identifier, const uint8_t *data, size_t data_length)
{
    twai_transport_t *transport = context;
    twai_transport_tx_slot_t *slot = NULL;

    if (transport == NULL || (data == NULL && data_length != 0) || data_length > TWAI_FRAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&transport->lock);
    for (size_t i = 0; i < TWAI_TRANSPORT_TX_POOL_SIZE; i++) {
        if (!transport->tx_slots[i].in_use) {
            slot = &transport->tx_slots[i];
            slot->in_use = true;
            break;
        }
    }
    portEXIT_CRITICAL(&transport->lock);
    if (slot == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (data_length != 0) {
        memcpy(slot->data, data, data_length);
    }
    slot->frame = (twai_frame_t) {
        .header.id = identifier,
        .header.ide = true,
        .buffer = slot->data,
        .buffer_len = data_length,
    };

    esp_err_t err = twai_node_transmit(transport->node, &slot->frame, 0);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&transport->lock);
        slot->in_use = false;
        portEXIT_CRITICAL(&transport->lock);
    }
    return err;
}
