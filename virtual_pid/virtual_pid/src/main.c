#include <stdio.h>
#include <stdlib.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "esc.h"
#include "pid.h"

#define REFERENCE_ALTITUDE 100
#define SEND_RATE_MS 10

#define LIFTOFF_THROTTLE 2092
#define THROTTLE_LIMIT 4000
#define MOTOR_NUM 4

struct sens_fb_st 
{
    int altitude;
} sens_fb;

K_THREAD_STACK_DEFINE(flight_stack, 2048);
static struct k_thread flight_tid;

pid_handler_t throttle_pid;
static int motor[MOTOR_NUM] = {0, 0, 0, 0};

const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
char uart_buffer[128];

static void sensor_reads(void);
static void send_motor_command(int m1, int m2, int m3, int m4);

#ifdef CONFIG_SIMULATION_MODE
static void uart_read(void);
#endif // CONFIG_SIMULATION_MODE

void flight_thread(void *a, void *b, void *c)
{
    static uint32_t time = 0;

    int feedback = 0;
    int control = 0;

    send_motor_command(0, 0, 0, 0);

    while (1)
    {
        sensor_reads();

        k_msleep(SEND_RATE_MS);

        feedback = atoi(uart_buffer);  // convert char to int

        control = pid_run(&throttle_pid, feedback, time);

        time += SEND_RATE_MS;

        printf("feedback = %d, control = %d\n", feedback, control);

        for (int i = 0; i < MOTOR_NUM; i++)
        {
            motor[i] = control + LIFTOFF_THROTTLE;

            if (motor[i] < 0)
                motor[i] = 0;
            else if (motor[i] > THROTTLE_LIMIT)
                motor[i] = THROTTLE_LIMIT;
        }

        send_motor_command(motor[0], motor[1], motor[2], motor[3]);

        fflush(stdout);
    }
}

int main(void)
{
    printf("Flight thread starting...\n");
    fflush(stdout);

    pid_init(&throttle_pid, REFERENCE_ALTITUDE, 21, 5, 8);
    esc_init(MOTOR_NUM);

    k_thread_create(&flight_tid, flight_stack,
                    K_THREAD_STACK_SIZEOF(flight_stack), flight_thread, NULL,
                    NULL, NULL, 5, 0, K_NO_WAIT);

    return 0;
}

static void sensor_reads(void)
{
#ifdef CONFIG_SIMULATION_MODE
    uart_read();
#else
    // Update sens_fb
    // sens_fb.altitude = read_buff();
#endif // CONFIG_SIMULATION_MODE
}

#ifdef #ifdef CONFIG_SIMULATION_MODE
/**
 * @brief Read a line from the console
 *
 */
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
#endif // CONFIG_SIMULATION_MODE

/**
 * @brief Send motor command to the console
 *
 * @param m1
 * @param m2
 * @param m3
 * @param m4
 */
static void send_motor_command(int m1, int m2, int m3, int m4)
{
#ifdef CONFIG_SIMULATION_MODE
    printf("%d.%03d %d.%03d %d.%03d %d.%03d\n", m1 / 1000, m1 % 1000, m2 / 1000,
           m2 % 1000, m3 / 1000, m3 % 1000, m4 / 1000, m4 % 1000);
#else
    float m[MOTOR_NUM] = {0.0f};

    m[1] = ((float)m1 / (float)THROTTLE_LIMIT);
    m[2] = ((float)m2 / (float)THROTTLE_LIMIT);
    m[3] = ((float)m3 / (float)THROTTLE_LIMIT);
    m[4] = ((float)m4 / (float)THROTTLE_LIMIT);

    esc_set(m);
#endif
}
