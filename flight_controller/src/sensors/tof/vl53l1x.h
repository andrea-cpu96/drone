#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdbool.h>

/**
 * @brief Result of a VL53L1X operation.
 */
enum vl53l1x_status
{
    VL53L1X_STATUS_OK,
    VL53L1X_STATUS_NOT_READY,
    VL53L1X_STATUS_I2C_TRANSFER_ERROR
};

/**
 * @brief Initialize the VL53L1X time-of-flight sensor.
 *
 * The Zephyr driver brings the device up at boot, so this only prepares the
 * local state.
 *
 * @return VL53L1X_STATUS_OK if the device is present and ready,
 *         VL53L1X_STATUS_NOT_READY otherwise.
 */
enum vl53l1x_status vl53l1x_init(void);

/**
 * @brief Read the latest distance measurement.
 *
 * @param[out] distance_mm measured distance in millimeters.
 *
 * @return VL53L1X_STATUS_OK on success, an error status otherwise.
 */
enum vl53l1x_status vl53l1x_read_distance(int *distance_mm);

#endif  // VL53L1X_H
