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

// Reference sea-level pressure (hPa) for the barometric altitude formula.
#define SEA_LEVEL_PRESSURE_HPA 1013.25f
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
    // Add a field here for every new sensor.
};

static struct sensor_samples_st sensor_sample;

static void barometer_init(void);
static int barometer_read(void);

void sensors_init(void)
{
    barometer_init();
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

/*
 * Initialize the MS5611 barometer: bring up the I2C master, verify the device is
 * present and reset it so the calibration PROM is reloaded into the internal
 * registers before the first measurement. Does nothing in simulation mode, where
 * the altitude comes from the UART.
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
#endif  // CONFIG_SIMULATION_MODE
}

/*
 * Read the MS5611 and convert the measured pressure into an altitude in meters.
 * The last published altitude is kept on a read error. Does nothing in simulation
 * mode, where the altitude comes from the UART.
 */
static int barometer_read(void)
{
#ifndef CONFIG_SIMULATION_MODE
    float temperature;
    float pressure;

    if (ms5611_read_temperature_and_pressure(&temperature, &pressure) != ms5611_status_ok)
    {
        return sensor_sample.altitude_sensor;
    }

    // Barometric formula: pressure (hPa) -> altitude (m) above sea level.
    return (int)(44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE_HPA, 1.0f / 5.255f)));
#else
    return sensor_sample.altitude_sensor;
#endif  // CONFIG_SIMULATION_MODE
}
