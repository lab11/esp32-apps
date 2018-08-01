# Time Sync

This example demonstrates time synchronization over BLE. 

The app connects to Wi-Fi to retrieve current time over SNTP, and then connects with compatible peripherals to update their clocks. 

The `peripheral` directory contains an example ESP32 app that acts as a compatible BLE device that connects to central for time synchronization.
