# Xiaomi CyberGear Driver

[![Component Registry](https://components.espressif.com/components/cybergear-robotics/cybergear/badge.svg)](https://components.espressif.com/components/cybergear-robotics/cybergear)
[![Examples build](https://github.com/cybergear-robotics/cybergear/actions/workflows/build_example.yml/badge.svg)](https://github.com/cybergear-robotics/cybergear/actions/workflows/build_example.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/cybergear-robotics/cybergear/commit-activity)
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-orange.svg)](https://shields.io/)
[![Language](https://img.shields.io/badge/Language-C-purple.svg)](https://shields.io/)


This driver uses Espressif's TWAI (Two-Wire Automotive Interface) in
order to communicate with Xiamoi CyberGear motors. It bases on the library
[Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino)
and is ported for ESP-IDF.

The component is independent of the CAN driver. Configure `cybergear_config_t::send`
with an adapter that sends one extended CAN frame; `send_context` is passed through to
that adapter. Incoming frames are routed by the application and passed to
`cybergear_process_message()` as `cybergear_message_t`.

## Safety

This library does not use error logs/prints, but instead every internal error is
passed through. Therefore each relevant function returns `esp_err_t`, which should
be handled. During development `ESP_ERROR_CHECK(...)` helps, but due to the strength
of these motors, an error should be resolved or the motors should be stopped by an
external emergency mechanism.

## Faults & Warnings

The motor provides a list of faults and warnings. If a fault is active, the
motor will sends a fault feedback frame. As the implementaiton is not completly
tested and not every fault occured yet, following list gives you a hint whether
a fault was already correctly tested:

### Faults
* [ ] `overload`
* [ ] `uncalibrated`
* [ ] `over_current_phase_a`
* [ ] `over_current_phase_b`
* [ ] `over_current_phase_c`
* [x] `over_voltage` (0x4)
* [ ] `under_voltage`
* [ ] `driver_chip`
* [ ] `over_temperature`
* [ ] `magnetic_code_failure`
* [ ] `hall_coded_faults`


### Warnings
* [ ] `over_temperature`

## Using component
```bash
idf.py add-dependency "cybergear-robotics/cybergear"
```

## Example

`position_test` shows one CyberGear motor. `shared_bus_test` adds a reusable TWAI
transport with an asynchronous TX frame pool and listener routing for additional CAN devices.

1. create example project
```bash
idf.py create-project-from-example "cybergear-robotics/cybergear:position_test"
```
2. Go to to example directory (for example `position_test`)
   `cd position_test`
3. Set ESP chip
   `idf.py set-target esp32`
4. Configure CAN TX/RX in menu `CyberGear Example`.
   `idf.py menuconfig`
5. Build, flash
   `idf.py build flash monitor`

## FAQ

### How to clear an alarm?

If a fault occured, the alarm cannot be cleared by the `cybergear_stop` command, which 
correspond to the `CMD_RESET`. I could not find a CAN command which cleares all alarms.
The CyberGear Dongle Tool provides a `Clear ALarm` button. The button is not tested yet.
The fault disappears after a power-cut.


### What if motor does not react to sent commands?

Register an `on_error` callback with `twai_node_register_event_callbacks()` to observe
transmission failures. These can have multiple reasons:

1. Too many messages are sent and the CAN TX Queue is to small. It helps to increase the
   queue length:
   ```
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX,
        .io_cfg.rx = RX,
        .bit_timing.bitrate = 1000000,
        .tx_queue_depth = 50,
    };
   ```

2. It seems that some other task (interupt handler, maybe?) needs to run sometimes in order
   to keep CAN message to be sent. More investigation is required.

## Related projects

* [Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino)
* [cybergear_m5](https://github.com/project-sternbergia/cybergear_m5)
