#include "sensors.h"

#include "i2c.h"

#ifdef CONFIG_SIMULATION_MODE
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
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

void sensors_altitude_process(void)
{
#ifdef CONFIG_SIMULATION_MODE
    if (uart_read())
    {
        sensor_sample.altitude_sensor = atoi(uart_buffer);
    }
#else
    // sensor_sample.altitude_sensor = read_buff();
#endif  // CONFIG_SIMULATION_MODE
}

int sensors_read_altitude(void)
{
    return sensor_sample.altitude_sensor;
}
