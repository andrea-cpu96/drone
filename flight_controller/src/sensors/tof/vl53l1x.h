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

enum vl53l1x_status vl53l1x_init(void);
enum vl53l1x_status vl53l1x_read_distance(int *distance_mm);

#endif  // VL53L1X_H
