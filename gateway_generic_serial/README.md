Serial Gateway for BLE Devices
==============================

Scans for BLE advertisements from any device and outputs resulting data over serial.

Baud rate for the serial console output is set to 921600. 

Using miniterm,

     python -m serial.tools.miniterm <port> 921600


Data Format
-----------

 * `6 bytes`: BLE Address
 * `4 bytes`: RSSI (signed)
 * `1 byte`: Event Type
 * `1 byte`: Remaining Length
 * `0-62 bytes`: Advertisement Data and Scan Response Data
 * `1 byte`: '\n'

Example output:

```
9801a7b9b6daa7ffffff001702010613ff4c000c0e00eda5639dd029af9a240fe8d97c
6c3e759f94bba6ffffff000e0201060aff4c0010054b1cbb05af
02ebac98b9c6a8ffffff031f1eff060001092002a1a4fd9808e87468b3a22920c14c8ae1a4aeb3d48f66d4
61219687d2e7a5ffffff000b02010607ff4c000f022033
55f078148f16a7ffffff001102011a020a180aff4c0010050a98c7b7d2
```


Notes:
 * Some startup junk will be printed before data begins. The last line before data is: "␛[0;32mI (889) SCAN: Start scanning...␛[0m".
 * All values in 2-digit-per-byte hexadecimal characters.
 * This is probably best used with pyserial (or similar) by reading lines.
 * Advertisement Data and Scan Response Data may or may not be concatenated together.
 * Event Types can be found here: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html#_CPPv418esp_ble_evt_type_t
 * I make no promises about the endianness of _anything_.
 * This guide may help: http://www-inst.eecs.berkeley.edu/~ee290c/sp18/lec/Lecture7A.pdf

