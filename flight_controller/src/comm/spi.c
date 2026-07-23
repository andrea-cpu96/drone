#include "spi.h"

#include "log.h"

#include <zephyr/drivers/spi.h>

// SPI bus controller resolved from the "spi-interface" devicetree alias. The
// bus is initialized automatically by Zephyr at boot, so this file only exposes
// a readiness check for the bus and the devices attached to it.
const struct device *const spi_driver = DEVICE_DT_GET(DT_ALIAS(spi_interface));

/**
 * @brief Verify the SPI bus controller and the given devices are ready.
 *
 * @param devices
 * @param count
 * @return bool
 */
bool spi_master_init(const struct device *const *devices, size_t count)
{
    if (!device_is_ready(spi_driver))
    {
        log_print("SPI device not ready\n");
        return false;
    }

    for (size_t i = 0; i < count; i++)
    {
        if (!device_is_ready(devices[i]))
        {
            log_print("SPI device not ready\n");
            return false;
        }
    }

    log_print("SPI master ready\n");
    return true;
}
