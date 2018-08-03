# Remote Firmware Update Example

This example receives a new application image "over-the-air" from a specified online location and runs it.

After building the application, serve the `build` directory. Enter the served IP address and port, as well as the appropriate Wi-Fi credentials in the 'Source & Wi-Fi Configuration' config menu.

When loading the initial application, make sure to completely erase flash, i.e.: 

    make erase_flash flash

Once started, the application should download the app image bin from the hosted `build` directory into the alternate partition. Once downloaded, the chip will restart and boot from the new image.


