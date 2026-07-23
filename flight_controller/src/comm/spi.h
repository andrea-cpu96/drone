#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>

bool spi_master_init(const struct device *const *devices, size_t count);

#endif  // SPI_H
