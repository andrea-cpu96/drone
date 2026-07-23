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

void log_init(struct k_msgq *queue);
void log_print(const char *fmt, ...) __printf_like(1, 2);

#endif  // LOG_H
