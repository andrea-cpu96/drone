#include "sensors.h"

#ifdef CONFIG_SIMULATION_MODE
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/uart.h>
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

#ifdef CONFIG_SIMULATION_MODE
static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static char uart_buffer[128];

static int uart_read(void);
#endif  // CONFIG_SIMULATION_MODE

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

#ifdef CONFIG_SIMULATION_MODE
/**
 * @brief Read a line from the console
 *
 */
static int uart_read(void)
{
    int data_ready = 0;

    uint8_t c;
    uint8_t byte_received = 0;

    memset(uart_buffer, 0, sizeof(uart_buffer));

    while (uart_poll_in(uart, &c) == 0)
    {
        if (c == '\n' || c == '\r')
        {
            uart_buffer[byte_received] = '\0';
            byte_received = 0;
            data_ready = 1;
        }
        else
        {
            if (byte_received < sizeof(uart_buffer) - 1)
            {
                uart_buffer[byte_received] = c;
                byte_received++;
            }
        }
    }
    return data_ready;
}
#endif  // CONFIG_SIMULATION_MODE
