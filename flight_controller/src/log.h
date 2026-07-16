#ifndef LOG_H
#define LOG_H

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>

// Maximum length in bytes of a single formatted log line (NUL terminator included).
#define LOG_MSG_MAX_LEN 128

// A single log entry as copied through the logging queue.
struct log_msg
{
    char text[LOG_MSG_MAX_LEN];
};

/**
 * @brief Initialize the logging module.
 *
 * Stores the message queue (owned by the caller) that log_print() fills and the
 * logging thread drains. Must be called before the first log_print().
 *
 * @param queue Message queue used to hand log entries to the logging thread.
 */
void log_init(struct k_msgq *queue);

/**
 * @brief Queue a formatted message for the logging thread.
 *
 * Behaves like printf(): the message is formatted and pushed into the logging
 * queue, to be printed later by the low-priority logging thread. If the queue
 * is full the message is dropped so the caller is never blocked. When
 * CONFIG_APP_LOG is disabled the call compiles to a no-op.
 *
 * @param fmt printf-style format string, followed by its arguments.
 */
void log_print(const char *fmt, ...) __printf_like(1, 2);

#endif  // LOG_H
