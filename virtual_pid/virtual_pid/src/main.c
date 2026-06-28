#include <stdio.h>
#include <stdlib.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "pid.h"

#define REFERENCE_ALTITUDE 100
#define SEND_RATE_MS 10 // send at 50 Hz

K_THREAD_STACK_DEFINE(flight_stack, 2048);
static struct k_thread flight_tid;

const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
char uart_buffer[128];

static void uart_read(void);

void flight_thread(void *a, void *b, void *c)
{
    static uint32_t time = 0;

    int m1, m2, m3, m4;
    int feedback = 0;
    int control = 0;

    printf("%d.%03d %d.%03d %d.%03d %d.%03d\n", 0, 0, 0, 0, 0, 0, 0, 0);

    while (1)
    {
        uart_read();

        k_msleep(SEND_RATE_MS);
        
        feedback = atoi(uart_buffer);  // convert char to int

        control = pid_run(feedback, time);

        time += SEND_RATE_MS;

        printf("feedback = %d, control = %d\n", feedback, control);

        m1 = control + 2092;
        m2 = control + 2092;
        m3 = control + 2092;
        m4 = control + 2092;

        if (m1 < 0)
            m1 = 0;
        else if (m1 > 4000)
            m1 = 4000;

        if (m2 < 0)
            m2 = 0;
        else if (m2 > 4000)
            m2 = 4000;
        
        if (m3 < 0)
            m3 = 0;
        else if (m3 > 4000)
            m3 = 4000;

        if (m4 < 0)
            m4 = 0;
        else if (m4 > 4000)
            m4 = 4000;

        //printf("m1: %d, m2: %d, m3: %d, m4: %d\n", m1, m2, m3, m4);

        printf("%d.%03d %d.%03d %d.%03d %d.%03d\n", m1 / 1000, m1 % 1000, m2 / 1000,
               m2 % 1000, m3 / 1000, m3 % 1000, m4 / 1000, m4 % 1000);

        fflush(stdout);
    }
}

int main(void)
{
    printf("Flight thread starting...\n");
    fflush(stdout);

    pid_init((pid_handler_t){.ref = REFERENCE_ALTITUDE, .kp = 21, .ki = 5, .kd = 8});

    k_thread_create(&flight_tid, flight_stack,
                    K_THREAD_STACK_SIZEOF(flight_stack), flight_thread, NULL,
                    NULL, NULL, 5, 0, K_NO_WAIT);

    return 0;
}

static void uart_read(void)
{
    uint8_t c;
    uint8_t byte_received = 0;

    while (1)
    {
        if (uart_poll_in(uart, &c) == 0)
        {
            if (c == '\n' || c == '\r')
            {
                uart_buffer[byte_received] = '\0';
                byte_received = 0;
                break;
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
    }
}