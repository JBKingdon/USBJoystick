# A simple USB HID Joystick implementation

Uses esp32 S2 or S3 for the USB interface and receives stick position info over esp-now from a suitable transmitter (no such transmitter is currently available, so you may want to hang on a bit before grabbing this). I'll put up an example transmitter project eventually.

Tested with S2 USB dongle, Lolin ESP32 S2 Mini and ESP32-S3-DevKitC-1. There aren't a lot of requirements for the hardware other than needing to be an S2 or S3 processor with the usb directly connected to the chip, not via a usb-serial adapter, and having an antenna for wifi.

## Example links from AliExpress:

https://www.aliexpress.com/item/1005006206868342.html

https://www.aliexpress.com/item/1005006157693055.html

https://www.aliexpress.com/item/1005006002965361.html

## Images:

![s2_mini](https://github.com/JBKingdon/USBJoystick/assets/12351913/62cd8a1f-5b0a-44e8-80cd-7f65994434dd)
![esp32-s3-devkitc-1-v1 1-isometric](https://github.com/JBKingdon/USBJoystick/assets/12351913/b34cef6f-f967-4afc-8c43-f84403cdbb14)
![dongle](https://github.com/JBKingdon/USBJoystick/assets/12351913/1374083e-eeba-4821-9033-20aa6203d7e6)
