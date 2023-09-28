# Arduino library for Eccel C1 family

This library provide simple interface for Eccel C1 family products by Eccel Technology Ltd.

## Examples

The library comes with two examples in File > Examples > Eccel-C1
- Get_uid- simple read UID from tags nerby antenna. Up to 5 tags can be read on one antenna (depends on antenna size etc.)
- Mifare_Classic - Read/Write eample for Mifare Classic tags
- Mifare_Ultralight_NTAG - Read/Write example for Mifare Ultralight/NTags2xx tags
- ICODE - Read/Write eample for Mifare Ultralight/NTags2xx tags
- ICODE_Inventory - ICODE inventory command example. Using inventory command the device can detect as many tags on the antenna as can be powered from hte device. On the build in antenna about 11-12 small 10mm tags can be detected.
- Desfire - eample for Mifare Ultralight/NTags2xx tags. Creating applications (folders), data files, credit files etc.


## Compatible Hardware

This library can be used with any C1 family product over UART connection. The recommended board is ESP32 based because examples use printf function and 2 hardware UARTS. But user can easily remove some lines to make it work on any hardware.

## License
Read LICENSE.txt for further informations on licensing terms.
