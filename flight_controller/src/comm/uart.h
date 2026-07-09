#ifndef UART_H
#define UART_H

#ifdef CONFIG_SIMULATION_MODE

extern char uart_buffer[128];

/**
 * @brief Read a line from the console
 *
 */
int uart_read(void);

#endif /* CONFIG_SIMULATION_MODE */

#endif /* UART_H */
