ESP32 Applications
==================

This repo contains applications for [ESP32](https://www.espressif.com/en/products/hardware/esp32/overview), an SoC with integrated BLE and Wi-Fi.

To prepare to run an application, [make sure the software environment is set up on your machine](https://esp-idf.readthedocs.io/en/latest/get-started/index.html), enter the app's directory, connect the device via USB, and [set the appropriate serial port](https://esp-idf.readthedocs.io/en/latest/get-started/index.html#configure in the configuration menu. 

If Internet access is necessary for the app, a custom setting labelled "Wi-Fi Configuration" should be present in the configuration menu to specify the Wi-Fi network and password. 

Then:

    make flash monitor

Hit `CTRL + ]` to halt serial monitoring.

The apps are tested on the [SparkFun ESP32 Thing](https://www.sparkfun.com/products/13907).
