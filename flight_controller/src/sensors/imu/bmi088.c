#include "bmi088.h"

#include "spi.h"

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

// The Zephyr BMI08x driver instantiates these devices from the
// "bosch,bmi08x-accel" and "bosch,bmi08x-gyro" devicetree nodes in the board
// overlay. DEVICE_DT_GET_ANY resolves to NULL when no such node exists (e.g. the
// QEMU/simulation build), so the code below stays compilable and simply reports
// the device as not ready.
static const struct device *const accel_dev = DEVICE_DT_GET_ANY(bosch_bmi08x_accel);
static const struct device *const gyro_dev = DEVICE_DT_GET_ANY(bosch_bmi08x_gyro);
static bool imu_ready;

/**
 * @brief Initialize the BMI088 inertial measurement unit.
 *
 * @return enum bmi088_status
 */
enum bmi088_status bmi088_init(void)
{
    const struct device *const devices[] = {accel_dev, gyro_dev};

    imu_ready = spi_master_init(devices, ARRAY_SIZE(devices));

    return imu_ready ? BMI088_STATUS_OK : BMI088_STATUS_NOT_READY;
}

/**
 * @brief Read the latest acceleration measurement (m/s^2).
 *
 * @param x
 * @param y
 * @param z
 * @return enum bmi088_status
 */
enum bmi088_status bmi088_read_accel(float *x, float *y, float *z)
{
    struct sensor_value accel[3];

    if (!imu_ready)
    {
        return BMI088_STATUS_NOT_READY;
    }

    if (sensor_sample_fetch(accel_dev) != 0)
    {
        return BMI088_STATUS_SPI_TRANSFER_ERROR;
    }

    if (sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel) != 0)
    {
        return BMI088_STATUS_SPI_TRANSFER_ERROR;
    }

    *x = (float)sensor_value_to_double(&accel[0]);
    *y = (float)sensor_value_to_double(&accel[1]);
    *z = (float)sensor_value_to_double(&accel[2]);
    return BMI088_STATUS_OK;
}

/**
 * @brief Read the latest angular rate measurement (rad/s).
 *
 * @param x
 * @param y
 * @param z
 * @return enum bmi088_status
 */
enum bmi088_status bmi088_read_gyro(float *x, float *y, float *z)
{
    struct sensor_value gyro[3];

    if (!imu_ready)
    {
        return BMI088_STATUS_NOT_READY;
    }

    if (sensor_sample_fetch(gyro_dev) != 0)
    {
        return BMI088_STATUS_SPI_TRANSFER_ERROR;
    }

    if (sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro) != 0)
    {
        return BMI088_STATUS_SPI_TRANSFER_ERROR;
    }

    *x = (float)sensor_value_to_double(&gyro[0]);
    *y = (float)sensor_value_to_double(&gyro[1]);
    *z = (float)sensor_value_to_double(&gyro[2]);
    return BMI088_STATUS_OK;
}
