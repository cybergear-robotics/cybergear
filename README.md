# Xiaomi CyberGear Driver

This driver uses Espressif's TWAI (Two-Wire Automotive Interface) in 
order to communicate with Xiamoi CyberGear motors. It bases on the library 
[Xiaomi_CyberGear_Arduino](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino) 
and is ported for ESP-IDF.


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