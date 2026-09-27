# Xiaomi CyberGear Driver

[![Component Registry](https://components.espressif.com/components/cybergear-robotics/cybergear/badge.svg)](https://components.espressif.com/components/cybergear-robotics/cybergear)
[![Examples build](https://github.com/cybergear-robotics/cybergear/actions/workflows/build_example.yml/badge.svg)](https://github.com/cybergear-robotics/cybergear/actions/workflows/build_example.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/cybergear-robotics/cybergear/commit-activity)
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-orange.svg)](https://shields.io/)
[![Language](https://img.shields.io/badge/Language-C-purple.svg)](https://shields.io/)


This driver uses Espressif's TWAI (Two-Wire Automotive Interface) to
communicate with Xiaomi CyberGear motors. It is based on the library
[Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino)
and has been ported to ESP-IDF.

The component is independent of the CAN driver. Configure `cybergear_config_t::send`
with an adapter that sends one extended CAN frame; `send_context` is passed through to
that adapter. Incoming frames are routed by the application and passed to
`cybergear_process_message()` as `cybergear_message_t`.

## Safety

This library does not log or print errors; instead, it propagates all internal errors.
Consequently, every relevant function returns an `esp_err_t` value that should be
handled. During development, `ESP_ERROR_CHECK(...)` is helpful. However, because
these motors are powerful, errors should be resolved or the motors stopped by an
external emergency mechanism.

## Faults & Warnings

The motor provides a list of faults and warnings. If a fault is active, the
motor sends a fault feedback frame. Since the implementation has not been fully
tested and not every fault has occurred yet, the following list indicates whether
a fault has been tested successfully:

### Faults
* [ ] `overload`
* [ ] `uncalibrated`
* [ ] `over_current_phase_a`
* [ ] `over_current_phase_b`
* [ ] `over_current_phase_c`
* [x] `over_voltage`
* [x] `under_voltage`
* [ ] `driver_chip`
* [ ] `over_temperature`
* [ ] `magnetic_code_failure`
* [ ] `hall_coded_faults`


### Warnings
* [ ] `over_temperature`

## Using the component
```bash
idf.py add-dependency "cybergear-robotics/cybergear"
```

## Example

1. Create an example project.
```bash
idf.py create-project-from-example "cybergear-robotics/cybergear:position_test"
```
2. Go to the example directory, for example `position_test`.
   `cd position_test`
3. Set the ESP target.
   `idf.py set-target esp32`
4. Configure CAN TX/RX in the `CyberGear Example` menu.
   `idf.py menuconfig`
5. Build, flash, and monitor the project.
   `idf.py build flash monitor`

## FAQ

### How to clear an alarm?

If a fault occurs, the alarm can be cleared with the `cybergear_stop` command,
which corresponds to `CMD_RESET`.


### What if the motor does not react to sent commands?

Register an `on_error` callback with `twai_node_register_event_callbacks()` to observe
transmission failures. These can have multiple reasons:

1. Too many messages are being sent, and the CAN TX queue is too small. Increasing
   the queue length may help:
   ```
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX,
        .io_cfg.rx = RX,
        .bit_timing.bitrate = 1000000,
        .tx_queue_depth = 50,
    };
   ```

2. Another task, possibly an interrupt handler, may need to run periodically for CAN
   messages to continue being sent. This requires further investigation.

## Related projects

* [Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino)
* [cybergear_m5](https://github.com/project-sternbergia/cybergear_m5)
