# PowerBlade to Thingspeak

Listens for BLE packets from a PowerBlade, parses the data, and periodically sends an updated reading to [Thingspeak](https://thingspeak.com/).

To comply with the service's limited quota, this example will only take readings from a single PowerBlade's broadcast and POST approximately every 15 seconds. Make sure to specify the PowerBlade's address and the API key for the destination ThingSpeak channel in config.