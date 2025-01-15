# Xiaomi CyberGear Driver

[![Examples build](https://github.com/cybergear-robotics/cybergear/actions/workflows/build_example.yml/badge.svg)](https://github.com/cybergear-robotics/cybergear/actions/workflows/build_example.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity)
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-orange.svg)](https://shields.io/)
[![Language](https://img.shields.io/badge/Language-C-purple.svg)](https://shields.io/)


This driver uses Espressif's TWAI (Two-Wire Automotive Interface) in 
order to communicate with Xiamoi CyberGear motors. It bases on the library 
[Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino) 
and is ported for ESP-IDF.

# Safety

This library does not use error logs/prints, but instead every internal error is 
passed through. Therefore each relevant function returns `esp_err_t`, which should
be handled. During development `ESP_ERROR_CHECK(...)` helps, but due to the strength
of these motors, an error should be resolved or the motors should be stopped by an 
external emergency mechanism.

Pay attention: The motor provides a list of faults. These are not tested yet and the code is beta.
Following lists the faults and whether a fault was already correctly tested:

* [ ] `overload`
* [ ] `uncalibrated`
* [ ] `over_current_phase_a`
* [ ] `over_current_phase_b`
* [ ] `over_current_phase_c`
* [ ] `over_voltage`
* [ ] `under_voltage`
* [ ] `driver_chip`
* [ ] `over_temperature`
* [ ] `magnetic_code_failure`
* [ ] `hall_coded_faults`


# Example

1. Clone Repository
2. Go to the example directory 
   `cd ./cybergear/examples/position_test`
3. Set ESP chip
   `idf.py set-target esp32`
4. Configure CAN TX/RX in menu `CyberGear Example`.
   `idf.py menuconfig`
5. Build, flash
   `idf.py build flash monitor`

# Related projects

* [Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino) 
* [cybergear_m5](https://github.com/project-sternbergia/cybergear_m5)