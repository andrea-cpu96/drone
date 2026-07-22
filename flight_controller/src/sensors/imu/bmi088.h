#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>

/**
 * @brief Result of a BMI088 operation.
 */
enum bmi088_status
{
    BMI088_STATUS_OK,
    BMI088_STATUS_NOT_READY,
    BMI088_STATUS_SPI_TRANSFER_ERROR
};

/**
 * @brief Initialize the BMI088 inertial measurement unit.
 *
 * The Zephyr driver brings the accelerometer and gyroscope up at boot, so this
 * only prepares the local state.
 *
 * @return BMI088_STATUS_OK if both devices are present and ready,
 *         BMI088_STATUS_NOT_READY otherwise.
 */
enum bmi088_status bmi088_init(void);

/**
 * @brief Read the latest acceleration measurement.
 *
 * @param[out] x acceleration on the X axis in m/s^2.
 * @param[out] y acceleration on the Y axis in m/s^2.
 * @param[out] z acceleration on the Z axis in m/s^2.
 *
 * @return BMI088_STATUS_OK on success, an error status otherwise.
 */
enum bmi088_status bmi088_read_accel(float *x, float *y, float *z);

/**
 * @brief Read the latest angular rate measurement.
 *
 * @param[out] x angular rate around the X axis in rad/s.
 * @param[out] y angular rate around the Y axis in rad/s.
 * @param[out] z angular rate around the Z axis in rad/s.
 *
 * @return BMI088_STATUS_OK on success, an error status otherwise.
 */
enum bmi088_status bmi088_read_gyro(float *x, float *y, float *z);

#endif  // BMI088_H
