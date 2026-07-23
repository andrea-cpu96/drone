#include "uart.h"

#ifdef CONFIG_SIMULATION_MODE

#include <stdint.h>
#include <string.h>

#include <zephyr/drivers/uart.h>

static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

char uart_buffer[128];

/**
 * @brief Read a line from the console
 *
 * @return int
 */
int uart_read(void)
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
