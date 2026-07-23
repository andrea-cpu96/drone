#include "vl53l1x.h"

#include "i2c.h"

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

// The Zephyr VL53L1X driver instantiates this device from the "st,vl53l1x"
// devicetree node in the board overlay. DEVICE_DT_GET_ANY resolves to NULL when
// no such node exists (e.g. the QEMU/simulation build), so the code below stays
// compilable and simply reports the device as not connected.
static const struct device *const tof_dev = DEVICE_DT_GET_ANY(st_vl53l1x);
static bool tof_ready;

/**
 * @brief Initialize the VL53L1X time-of-flight sensor.
 *
 * @return enum vl53l1x_status
 */
enum vl53l1x_status vl53l1x_init(void)
{
    const struct device *const devices[] = {tof_dev};

    tof_ready = i2c_master_init(devices, ARRAY_SIZE(devices));

    return tof_ready ? VL53L1X_STATUS_OK : VL53L1X_STATUS_NOT_READY;
}

/**
 * @brief Read the latest distance measurement (millimeters).
 *
 * @param distance_mm
 * @return enum vl53l1x_status
 */
enum vl53l1x_status vl53l1x_read_distance(int *distance_mm)
{
    struct sensor_value distance;

    if (!tof_ready)
    {
        return VL53L1X_STATUS_NOT_READY;
    }

    if (sensor_sample_fetch(tof_dev) != 0)
    {
        return VL53L1X_STATUS_I2C_TRANSFER_ERROR;
    }

    if (sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &distance) != 0)
    {
        return VL53L1X_STATUS_I2C_TRANSFER_ERROR;
    }

    // This driver stores the raw range in millimeters in
    // val1 (val2 is always 0).
    *distance_mm = distance.val1;
    return VL53L1X_STATUS_OK;
}
