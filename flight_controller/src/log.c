#include "log.h"

#include <stdarg.h>
#include <stdio.h>

// Queue (owned by the caller) used to hand log entries to the logging thread.
static struct k_msgq *log_queue;

/**
 * @brief Initialize the logging module.
 *
 * @param queue Message queue used to hand log entries to the logging thread.
 */
void log_init(struct k_msgq *queue)
{
    log_queue = queue;
}

/**
 * @brief Queue a formatted message for the logging thread.
 *
 * @param fmt printf-style format string, followed by its arguments.
 */
void log_print(const char *fmt, ...)
{
#ifdef CONFIG_APP_LOG
    struct log_msg msg;
    va_list args;

    if (log_queue == NULL)
    {
        return;
    }

    va_start(args, fmt);
    vsnprintf(msg.text, sizeof(msg.text), fmt, args);
    va_end(args);

    // Non-blocking put: drop the message when the queue is full so that logging
    // never blocks the (possibly real-time) calling thread.
    k_msgq_put(log_queue, &msg, K_NO_WAIT);
#else
    (void)fmt;
#endif
}
