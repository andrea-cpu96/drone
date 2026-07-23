#include "sensors.h"

#include "i2c.h"
#include "log.h"

#ifdef CONFIG_SIMULATION_MODE

#include <stdint.h>
#include <stdlib.h>

#include "uart.h"

#else

#include <math.h>

#include <zephyr/kernel.h>

#include "bmi088.h"
#include "ms5611.h"
#include "vl53l1x.h"

// Reference sea-level pressure (hPa) for the barometric altitude formula.
#define SEA_LEVEL_PRESSURE_HPA 1013.25f

// Number of altitude samples averaged at init to capture the ground reference.
#define BAROMETER_ZERO_SAMPLES 20

// Below this altitude (m) the barometer is unreliable near the ground, so the
// ToF distance is used as the altitude instead.
#define ALTITUDE_TOF_THRESHOLD_M 3

#endif  // CONFIG_SIMULATION_MODE

// A 3-axis sensor reading (accelerometer in m/s^2, gyroscope in rad/s).
struct axes_st
{
    float x;
    float y;
    float z;
};

/*
 * State of all sensors, shared between the sensors thread (writer, through
 * sensors_*_process) and the control thread (reader).
 *
 * The scalar fields are word-sized and written with a single store, so on the
 * target the access is atomic: the reader always observes either the previous
 * or the new complete value, never a partial one. The 3-axis fields
 * (accelerometer, gyroscope) are written one component at a time, so a reader
 * may briefly observe a mix of the previous and new vector; that is acceptable
 * for the control loop, which re-reads every cycle. 'volatile' prevents the
 * compiler from caching the shared values.
 */
struct sensor_samples_st
{
    volatile float altitude_sensor;         // meters
    volatile struct axes_st accelerometer;  // m/s^2
    volatile struct axes_st gyroscope;      // rad/s
    // Add a field here for every new sensor.
};

static struct sensor_samples_st sensor_sample;

#ifndef CONFIG_SIMULATION_MODE
// Ground reference altitude (meters) captured by barometer_init and subtracted
// from every barometer_read so the published altitude is relative to takeoff.
static float barometer_altitude_zero = 0.0f;
#endif  // CONFIG_SIMULATION_MODE

static void barometer_init(void);
static void tof_init(void);
static void imu_init(void);
#ifndef CONFIG_SIMULATION_MODE
static float barometer_read(void);
static bool barometer_read_absolute(float *altitude_m);
static bool tof_read(float *altitude_m);
static bool imu_read_accel(struct axes_st *accel);
static bool imu_read_gyro(struct axes_st *gyro);
#endif  // CONFIG_SIMULATION_MODE

/**
 * @brief Initialize all sensors (barometer, ToF, IMU).
 */
void sensors_init(void)
{
    barometer_init();
    tof_init();
    imu_init();
    // Add the init of every new sensor here.
}

/**
 * @brief Read the altitude source and publish the latest value.
 */
void sensors_altitude_process(void)
{
#ifdef CONFIG_SIMULATION_MODE
    if (uart_read())
    {
        // The simulator sends the altitude in meters.
        sensor_sample.altitude_sensor = (float)atof(uart_buffer);
    }
#else
    // Near the ground the barometer is unreliable, so below the threshold the
    // more accurate ToF distance is used as the altitude (both in meters).
    float barometer_m = barometer_read();

    if (barometer_m < ALTITUDE_TOF_THRESHOLD_M)
    {
        float tof_m;

        if (tof_read(&tof_m))
        {
            sensor_sample.altitude_sensor = tof_m;
        }
        else
        {
            sensor_sample.altitude_sensor = barometer_m;
        }
    }
    else
    {
        sensor_sample.altitude_sensor = barometer_m;
    }
#endif  // CONFIG_SIMULATION_MODE
}

/**
 * @brief Get the latest published altitude value.
 *
 * @return float
 */
float sensors_read_altitude(void)
{
    return sensor_sample.altitude_sensor;
}

/**
 * @brief Read the IMU (accelerometer + gyroscope) and publish the latest values.
 */
void sensors_imu_process(void)
{
#ifndef CONFIG_SIMULATION_MODE
    struct axes_st accel;
    struct axes_st gyro;

    if (imu_read_accel(&accel))
    {
        sensor_sample.accelerometer.x = accel.x;
        sensor_sample.accelerometer.y = accel.y;
        sensor_sample.accelerometer.z = accel.z;
    }

    if (imu_read_gyro(&gyro))
    {
        sensor_sample.gyroscope.x = gyro.x;
        sensor_sample.gyroscope.y = gyro.y;
        sensor_sample.gyroscope.z = gyro.z;
    }
#endif  // CONFIG_SIMULATION_MODE
}

/**
 * @brief Initialize the MS5611 barometer and capture the ground reference.
 *
 * Bring up the I2C master, verify the device is present and reset it so the
 * calibration PROM is reloaded into the internal registers before the first
 * measurement. Then capture the ground reference by averaging
 * BAROMETER_ZERO_SAMPLES altitude readings. Does nothing in simulation mode,
 * where the altitude comes from the UART.
 */
static void barometer_init(void)
{
#ifndef CONFIG_SIMULATION_MODE
    ms5611_init();

    if (!ms5611_is_connected())
    {
        log_print("MS5611 barometer not detected\n");
        return;
    }

    if (ms5611_reset() != ms5611_status_ok)
    {
        log_print("MS5611 reset failed\n");
        return;
    }

    // Wait for the calibration PROM to be reloaded after the reset.
    k_msleep(3);

    // Capture the ground reference: average BAROMETER_ZERO_SAMPLES absolute
    // altitude readings and store it as the zero subtracted on every read.
    float altitude_sum = 0.0f;

    for (int i = 0; i < BAROMETER_ZERO_SAMPLES; i++)
    {
        float altitude;

        if (!barometer_read_absolute(&altitude))
        {
            log_print("MS5611 zero calibration failed\n");
            return;
        }

        altitude_sum += altitude;
    }

    barometer_altitude_zero = altitude_sum / BAROMETER_ZERO_SAMPLES;

    log_print("MS5611 barometer ready (zero = %.2f m)\n", (double)barometer_altitude_zero);
#endif  // CONFIG_SIMULATION_MODE
}

/**
 * @brief Initialize the VL53L1X time-of-flight sensor.
 *
 * Does nothing in simulation mode, where there is no distance source.
 */
static void tof_init(void)
{
#ifndef CONFIG_SIMULATION_MODE
    if (vl53l1x_init() != VL53L1X_STATUS_OK)
    {
        log_print("VL53L1X ToF sensor not detected\n");
    }
    else
    {
        log_print("VL53L1X ToF ready\n");
    }
#endif  // CONFIG_SIMULATION_MODE
}

/**
 * @brief Initialize the BMI088 IMU (accelerometer + gyroscope).
 *
 * Does nothing in simulation mode, where there is no IMU source.
 */
static void imu_init(void)
{
#ifndef CONFIG_SIMULATION_MODE
    if (bmi088_init() != BMI088_STATUS_OK)
    {
        log_print("BMI088 IMU not detected\n");
    }
    else
    {
        log_print("BMI088 IMU ready\n");
    }
#endif  // CONFIG_SIMULATION_MODE
}

#ifndef CONFIG_SIMULATION_MODE
/**
 * @brief Read the barometer as a takeoff-relative altitude in meters.
 *
 * The last published altitude is kept on a read error.
 *
 * @return float
 */
static float barometer_read(void)
{
    float altitude;

    if (!barometer_read_absolute(&altitude))
    {
        return sensor_sample.altitude_sensor;
    }

    // Takeoff-relative altitude, in meters.
    return altitude - barometer_altitude_zero;
}

/**
 * @brief Read the MS5611 and compute the absolute altitude above sea level.
 *
 * @param altitude_m
 * @return bool
 */
static bool barometer_read_absolute(float *altitude_m)
{
    float temperature;
    float pressure;

    if (ms5611_read_temperature_and_pressure(&temperature, &pressure) != ms5611_status_ok)
    {
        return false;
    }

    // Barometric formula: pressure (hPa) -> altitude (m) above sea level.
    *altitude_m = 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE_HPA, 1.0f / 5.255f));
    return true;
}

/**
 * @brief Read the VL53L1X distance (in meters) via the out parameter.
 *
 * Returns false on a read error; the caller falls back to the barometer.
 *
 * @param altitude_m
 * @return bool
 */
static bool tof_read(float *altitude_m)
{
    int distance_mm;

    if (vl53l1x_read_distance(&distance_mm) != VL53L1X_STATUS_OK)
    {
        return false;
    }

    // The driver reports millimeters; convert to meters.
    *altitude_m = distance_mm / 1000.0f;
    return true;
}

/**
 * @brief Read the BMI088 accelerometer.
 *
 * @param accel
 * @return bool
 */
static bool imu_read_accel(struct axes_st *accel)
{
    return bmi088_read_accel(&accel->x, &accel->y, &accel->z) == BMI088_STATUS_OK;
}

/**
 * @brief Read the BMI088 gyroscope.
 *
 * @param gyro
 * @return bool
 */
static bool imu_read_gyro(struct axes_st *gyro)
{
    return bmi088_read_gyro(&gyro->x, &gyro->y, &gyro->z) == BMI088_STATUS_OK;
}
#endif  // CONFIG_SIMULATION_MODE
