#include "sensors.h"

#include "i2c.h"

#ifdef CONFIG_SIMULATION_MODE
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#else
#include <math.h>
#include <stdio.h>

#include <zephyr/kernel.h>

#include "ms5611.h"
#include "vl53l1x.h"

// Reference sea-level pressure (hPa) for the barometric altitude formula.
#define SEA_LEVEL_PRESSURE_HPA 1013.25f

// Number of altitude samples averaged at init to capture the ground reference.
#define BAROMETER_ZERO_SAMPLES 10
#endif  // CONFIG_SIMULATION_MODE

/*
 * State of all sensors, shared between the sensors thread (writer, through
 * sensors_altitude_process) and the control thread (reader, through
 * sensors_read_altitude).
 *
 * Each field is a word-sized scalar written with a single store, so on the
 * target the access is atomic: the reader always observes either the previous
 * or the new complete value, never a partial one. No lock or "updating" flag is
 * needed. 'volatile' prevents the compiler from caching the shared value.
 */
struct sensor_samples_st
{
    volatile int altitude_sensor;
    volatile int distance_sensor;
    // Add a field here for every new sensor.
};

static struct sensor_samples_st sensor_sample;

#ifndef CONFIG_SIMULATION_MODE
// Ground reference altitude (meters) captured by barometer_init and subtracted
// from every barometer_read so the published altitude is relative to takeoff.
static float barometer_altitude_zero = 0.0f;
#endif  // CONFIG_SIMULATION_MODE

static void barometer_init(void);
static int barometer_read(void);
#ifndef CONFIG_SIMULATION_MODE
static bool barometer_read_absolute(float *altitude_m);
#endif  // CONFIG_SIMULATION_MODE
static void tof_init(void);
static int tof_read(void);

void sensors_init(void)
{
    barometer_init();
    tof_init();
    // Add the init of every new sensor here.
}

void sensors_altitude_process(void)
{
#ifdef CONFIG_SIMULATION_MODE
    if (uart_read())
    {
        sensor_sample.altitude_sensor = atoi(uart_buffer);
    }
#endif  // CONFIG_SIMULATION_MODE

    sensor_sample.altitude_sensor = barometer_read();
}

int sensors_read_altitude(void)
{
    return sensor_sample.altitude_sensor;
}

void sensors_distance_process(void)
{
    sensor_sample.distance_sensor = tof_read();
}

int sensors_read_distance(void)
{
    return sensor_sample.distance_sensor;
}

/*
 * Initialize the MS5611 barometer: bring up the I2C master, verify the device is
 * present and reset it so the calibration PROM is reloaded into the internal
 * registers before the first measurement. Then capture the ground reference by
 * averaging BAROMETER_ZERO_SAMPLES altitude readings. Does nothing in simulation
 * mode, where the altitude comes from the UART.
 */
static void barometer_init(void)
{
#ifndef CONFIG_SIMULATION_MODE
    ms5611_init();

    if (!ms5611_is_connected())
    {
        printf("MS5611 barometer not detected\n");
        return;
    }

    if (ms5611_reset() != ms5611_status_ok)
    {
        printf("MS5611 reset failed\n");
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
            printf("MS5611 zero calibration failed\n");
            return;
        }

        altitude_sum += altitude;
    }

    barometer_altitude_zero = altitude_sum / BAROMETER_ZERO_SAMPLES;
#endif  // CONFIG_SIMULATION_MODE
}

/*
 * Read the MS5611 and convert the measured pressure into an altitude in meters,
 * relative to the ground reference captured at init. The last published altitude
 * is kept on a read error. Does nothing in simulation mode, where the altitude
 * comes from the UART.
 */
static int barometer_read(void)
{
#ifndef CONFIG_SIMULATION_MODE
    float altitude;

    if (!barometer_read_absolute(&altitude))
    {
        return sensor_sample.altitude_sensor;
    }

    // Subtract the ground reference captured at init so the result is relative
    // to takeoff.
    return (int)(altitude - barometer_altitude_zero);
#else
    return sensor_sample.altitude_sensor;
#endif  // CONFIG_SIMULATION_MODE
}

#ifndef CONFIG_SIMULATION_MODE
/*
 * Read the MS5611 and compute the absolute altitude in meters above sea level.
 * Returns false on an I2C/read error.
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
#endif  // CONFIG_SIMULATION_MODE

/*
 * Initialize the VL53L1X time-of-flight sensor. Does nothing in simulation mode,
 * where there is no distance source.
 */
static void tof_init(void)
{
#ifndef CONFIG_SIMULATION_MODE
    if (vl53l1x_init() != VL53L1X_STATUS_OK)
    {
        printf("VL53L1X ToF sensor not detected\n");
    }
#endif  // CONFIG_SIMULATION_MODE
}

/*
 * Read the VL53L1X and return the measured distance in millimeters. The last
 * published distance is kept on a read error. Does nothing in simulation mode,
 * where there is no distance source.
 */
static int tof_read(void)
{
#ifndef CONFIG_SIMULATION_MODE
    int distance_mm;

    if (vl53l1x_read_distance(&distance_mm) != VL53L1X_STATUS_OK)
    {
        return sensor_sample.distance_sensor;
    }

    return distance_mm;
#else
    return sensor_sample.distance_sensor;
#endif  // CONFIG_SIMULATION_MODE
}
