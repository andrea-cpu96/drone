#include <stdio.h>
#include <stdlib.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "pid.h"

#define REFERENCE_ALTITUDE 1000

#define SEND_RATE_MS 250 // send at 50 Hz
#define T_CLIMB_MS 1020
// Motor commands in milli-units (x1000)
#define CLIMB_MILLI 3000  // climb (strong thrust ~2:1)

K_THREAD_STACK_DEFINE(flight_stack, 2048);
static struct k_thread flight_tid;

const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
char uart_buffer[128];

static void uart_read(void);

void flight_thread(void *a, void *b, void *c)
{
    int m1, m2, m3, m4;
    int feedback = 0;
    int control = 0;

    // climb
    m1 = m2 = m3 = m4 = CLIMB_MILLI;

    printf("%d.%03d %d.%03d %d.%03d %d.%03d\n", 0, 0, 0, 0, 0, 0, 0, 0);

    while (1)
    {
        uart_read();

        k_msleep(SEND_RATE_MS);

        feedback = atoi(uart_buffer);  // convert char to int

        control = pid_run(feedback, 0);

        if (control < -1000)
            control = -1000;

        if (control > 1000)
            control = 1000;

        printf("feedback = %d, control = %d\n", feedback, control);

        m1 += control;
        m2 += control;
        m3 += control;
        m4 += control;

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

    pid_init((pid_handler_t){.ref = REFERENCE_ALTITUDE, .kp = 10, .ki = 0, .kd = 0});

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