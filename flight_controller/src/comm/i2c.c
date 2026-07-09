#include "i2c.h"

#include <zephyr/drivers/i2c.h>

const struct device *const i2c_driver =
    DEVICE_DT_GET(DT_ALIAS(i2c_interface));

