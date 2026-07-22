#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>

// Verify the SPI bus controller referenced by the "spi-interface" devicetree
// alias and the given SPI peripheral devices are all ready, and log the result.
// The bus and the device drivers are initialized automatically by Zephyr at
// boot, so this is only a readiness check / diagnostic.
//
// @param devices Array of SPI bus devices to verify (may be NULL if count 0).
// @param count   Number of entries in 'devices'.
// @return true if the bus and every listed device are ready, false otherwise.
bool spi_master_init(const struct device *const *devices, size_t count);

#endif  // SPI_H
