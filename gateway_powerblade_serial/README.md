Serial Gateway for PowerBlade
=============================

Scans for BLE advertisements from PowerBlades, translates packet data into
measurements, and outputs results over serial.

Baud rate for the serial console output is set to 921600. 

Using miniterm,

     python -m serial.tools.miniterm <port> 921600

