# Driver Notes

This document records behavior of this ESP-IDF driver which is not described
by the Xiaomi protocol manual.

## ESP-IDF Tests

The hardware tests use the standard ESP-IDF component-test layout:

```
test/                         # cybergear component test sources and Kconfig
test/test_cybergear_hardware.c
test/CMakeLists.txt
test_app/                     # minimal ESP-IDF test application
test_app/main/test_runner.c
```

`test_app/CMakeLists.txt` sets `TEST_COMPONENTS=cybergear`; ESP-IDF builds
`test/` as the `cybergear_test` component. The runner calls
`unity_run_all_tests()` after boot.

Run the suite from `test_app`:

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

The checked-in hardware defaults are CAN TX GPIO 13 and CAN RX GPIO 14.
The test scans node IDs with status requests before initializing the driver,
which permits recovery from an incorrect configured motor CAN ID.

## Type-0 Device Info

Type-0 responses are decoded into `cybergear_device_info_t` and retrieved with
`cybergear_get_device_info()`.

| Field | Source |
| --- | --- |
| `motor_can_id` | Identifier bits 15:8 |
| `recipient_can_id` | Identifier bits 7:0 |
| `unique_id[8]` | Eight raw data bytes |
| `updated` | Set after a valid type-0 frame is decoded |

The motor sends this response after `cybergear_set_motor_can_id()`, including
when the requested ID is unchanged. The response recipient is `0xFE`, the
protocol broadcast value, not the configured host CAN ID.

## Driver Behavior

- Commands report CAN transport submission only. Motor replies are asynchronous
  and must be passed to `cybergear_process_message()`.
- `cybergear_stop()` sends `CMD_RESET` with data byte 0 set to `0`; it stops
  the motor without clearing active faults.
- `cybergear_stop_and_clear_faults()` sends `CMD_RESET` with data byte 0 set
  to `1`; it stops the motor and clears active faults.
- This changed the behavior of `cybergear_stop()`. Existing applications that
  rely on it to clear faults must call `cybergear_stop_and_clear_faults()`.
- `cybergear_get_status()` and `cybergear_get_faults()` return cached data. A
  caller must wait for a new type-2 or type-21 frame when fresh data is needed.
- Parameter reads share one `params.updated` flag and have no request sequence
  number. Serialize requests and verify the returned parameter address.
- The current-limit API accepts up to 27 A, although the manual specifies 23 A
  as the maximum for position/speed current limiting. Applications should cap
  their configured limit at 23 A.

## Observed Firmware Limits

The tested motor answered reads for addresses `0x7019` through `0x7020`, but
the returned values were not plausible telemetry: position, filtered current,
velocity, bus voltage, and several gains were reported as `1.0` or `0`.
Use type-2 status feedback for position, speed, torque, and temperature until
these extended parameters are validated on the target motor firmware.
