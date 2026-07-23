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

enum bmi088_status bmi088_init(void);
enum bmi088_status bmi088_read_accel(float *x, float *y, float *z);
enum bmi088_status bmi088_read_gyro(float *x, float *y, float *z);

#endif  // BMI088_H
