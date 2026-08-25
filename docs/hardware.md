# Hardware

## ESP32-P4/S3

32-bit microcontrollers developed by Espressif Systems. These particular variants are dual-core, allowing two executions to operate in parallel.

There are different types of memories in these modules:

**SRAM**: Internal RAM, volatile, fastest.

**PSRAM**: Additional RAM, but slower than SRAM since it connects to the processor via SPI.

**FLASH**: SPI Flash is non-volatile meaning it survives powering off. This is where firmware is written. It is the slowest of the bunch. You can store variable data in flash by creating an NVS partition; the ESP32 can write to it and store things like configuration values not pre-built into the firmware.

You can typically find the avaible memory in the module name. 
The *ESP32-S3-WROOM-1-N8R2* has 8MB of flash and 2MB of PSRAM.

SRAM on a p4/s3: (768KB/512KB)

Be aware that having 8MB or more of PSRAM may comprimise GPIO pins.

## ESP32-C3/C6

These other modules are relevant and have different capabilities/specifications.


*Hand written by Edgar VM, Osiris Avionics Lead - Aug 24, 2026*