# Influx Configuration Example 

Sets up a simple HTTP server on the ESP32 that is accessible locally on the connected Wi-Fi network. 

A configuration page is served at the device's IP address and should be viewable from any browser on the network.

The address is advertised as an mDNS service on the network, in addition to being logged on the serial monitor during startup.

Additionally, the ESP32 listens for PowerBlade BLE packets, parses the data, and sends the data in batches to an InfluxDB endpoint.

The InfluxDB endpoint is configurable on the served webpage.
