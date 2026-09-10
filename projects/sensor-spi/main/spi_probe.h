// spi_probe.h
//
// Bring-up diagnostics. Project-specific and throwaway: once the link is
// known good this can be deleted along with its call in app_main().

#pragma once

#include "ai_spi.h"

// Runs before the bus is handed to the driver. Checks the ESP side of each
// pin, decides whether anything is driving MISO, then reads WHO_AM_I in all
// four SPI modes. Leaves the bus and pins released.
void spi_probe_run(const ai_spi_bus_config_t *bus_config,
                   const ai_spi_device_config_t *device_config);
