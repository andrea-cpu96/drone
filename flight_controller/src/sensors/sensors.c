#include "sensors.h"

#include "i2c.h"

#include <zephyr/kernel.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ms5611.h"

#ifdef CONFIG_SIMULATION_MODE
#include "uart.h"
#endif

// Reference sea-level pressure (hPa) for the barometric altitude formula.
#define SEA_LEVEL_PRESSURE_HPA 1013.25f

#define TEST_ALTITUDE_COUNT 3
#define TEST_SEND_DELAY_MS 10000U
#define TEST_I2C_ADDRESS 0x77U

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
static void test_send_raw_i2c_payload(void);

static int test_altitudes[TEST_ALTITUDE_COUNT];
static int test_altitude_index;
static bool test_altitudes_captured;
static bool test_altitudes_sent;
static int64_t test_send_start_ms;

static void test_send_raw_i2c_payload(void)
{
    if (test_altitude_index < TEST_ALTITUDE_COUNT || test_altitudes_sent)
    {
        return;
    }

    uint8_t payload[TEST_ALTITUDE_COUNT * 2];
    for (int i = 0; i < TEST_ALTITUDE_COUNT; i++)
    {
        uint16_t altitude = (uint16_t)test_altitudes[i];
        payload[2 * i] = (uint8_t)(altitude >> 8);
        payload[2 * i + 1] = (uint8_t)(altitude & 0xFF);
    }

    struct i2c_master_packet packet = {
        .address = TEST_I2C_ADDRESS,
        .data_length = sizeof(payload),
        .data = payload,
    };

    /* Send raw bytes to the sensor regardless of ACK. */
    (void)i2c_master_write_packet_wait(&packet);
    test_altitudes_sent = true;
}

void sensors_init(void)
{
    barometer_init();
}

void sensors_altitude_process(void)
{
    if (!test_altitudes_captured)
    {
#ifdef CONFIG_SIMULATION_MODE
        if (uart_read())
        {
            sensor_sample.altitude_sensor = atoi(uart_buffer);
        }
        else
#endif  // CONFIG_SIMULATION_MODE
        {
            sensor_sample.altitude_sensor = barometer_read();
        }

        if (test_altitude_index < TEST_ALTITUDE_COUNT)
        {
            test_altitudes[test_altitude_index++] = sensor_sample.altitude_sensor;
            if (test_altitude_index >= TEST_ALTITUDE_COUNT)
            {
                test_altitudes_captured = true;
                test_send_start_ms = k_uptime_get();
            }
        }
    }
    else if (!test_altitudes_sent)
    {
        if (k_uptime_get() - test_send_start_ms >= TEST_SEND_DELAY_MS)
        {
            test_send_raw_i2c_payload();
        }
    }
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
