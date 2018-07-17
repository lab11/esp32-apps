# Simple Forwarding UDP

Scans for BLE devices and sends received advertisement data to an endpoint via UDP.

Make sure to specify the target UDP address and port, as well as the Wi-Fi network and password, in the configuration menu.

To set up a test UDP endpoint on your machine, you can use netcat:

    nc -u -l <PORT NUMBER>

Then, add your IP address and the port number you specified above in the UDP Endpoint setting of the configuration menu.
