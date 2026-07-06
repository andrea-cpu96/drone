#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "esc.h"
#include "pid.h"

#define REFERENCE_ALTITUDE 100

#define CONTROL_THREAD_PERIOD_MS 10
#define SENSORS_THREAD_PERIOD_MS 100

#define LIFTOFF_THROTTLE 2092
#define THROTTLE_LIMIT 4000
#define MOTOR_NUM 4

// Controller thread configuration
#define PID_THREAD_PRIO     1
K_THREAD_STACK_DEFINE(controller_stack, 2048);
static struct k_thread controller_tid;
// Sensors thread configuration
#define SENSORS_THREAD_PRIO 5
K_THREAD_STACK_DEFINE(sensors_stack, 2048);
static struct k_thread sensors_tid;

// A single sensor reading: its value plus a dedicated flag set while the value
// is being written, so the controller can skip the read instead of blocking and
// never observe a partially updated value.
struct sensor_st
{
    int value;
    volatile bool updating;
};

struct sensor_fb_st
{
    struct sensor_st altitude;
} sens_fb;

pid_handler_t throttle_pid;
static int motor[MOTOR_NUM] = {0, 0, 0, 0};

const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
char uart_buffer[128];

static void sensor_reads(void);
static void read_altitude(void);
static void send_motor_command(int m1, int m2, int m3, int m4);
static void sensors_thread(void *a, void *b, void *c);

#ifdef CONFIG_SIMULATION_MODE
static int uart_read(void);
#endif  // CONFIG_SIMULATION_MODE

void controller_thread(void *a, void *b, void *c)
{
    static uint32_t time = 0;

    int altitude_feedback = 0;
    int control = 0;

    send_motor_command(0, 0, 0, 0);

    while (1)
    {
        k_msleep(CONTROL_THREAD_PERIOD_MS);

        /*
        * Read the feedback produced by the sensors thread. If the altitude is
        * being updated, keep the previous feedback instead of blocking, so we
        * never read a partially updated value.
        */
        if (!sens_fb.altitude.updating)
        {
            altitude_feedback = sens_fb.altitude.value;
        }

        control = pid_run(&throttle_pid, altitude_feedback, time);

        time += CONTROL_THREAD_PERIOD_MS;

        printf("feedback = %d, control = %d\n", altitude_feedback, control);

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

static void sensors_thread(void *a, void *b, void *c)
{
    while (1)
    {
        sensor_reads();

        k_msleep(SENSORS_THREAD_PERIOD_MS);
    }
}

int main(void)
{
    printf("Controller thread starting...\n");
    fflush(stdout);

    pid_init(&throttle_pid, REFERENCE_ALTITUDE, 21, 5, 8);
    esc_init(MOTOR_NUM);

    k_thread_create(&controller_tid, controller_stack,
                    K_THREAD_STACK_SIZEOF(controller_stack), controller_thread, NULL,
                    NULL, NULL, PID_THREAD_PRIO, 0, K_NO_WAIT);

    k_thread_create(&sensors_tid, sensors_stack,
                    K_THREAD_STACK_SIZEOF(sensors_stack), sensors_thread, NULL,
                    NULL, NULL, SENSORS_THREAD_PRIO, 0, K_NO_WAIT);

    return 0;
}

static void sensor_reads(void)
{
    read_altitude();
    // Add a dedicated read function here for every new sensor.
}

static void read_altitude(void)
{
#ifdef CONFIG_SIMULATION_MODE
    if(uart_read())
    {
        sens_fb.altitude.updating = true;
        sens_fb.altitude.value = atoi(uart_buffer);
        sens_fb.altitude.updating = false;
    }
#else
    // sens_fb.altitude.updating = true;
    // sens_fb.altitude.value = read_buff();
    // sens_fb.altitude.updating = false;
#endif  // CONFIG_SIMULATION_MODE
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

    m[0] = ((float)m1 / (float)THROTTLE_LIMIT);
    m[1] = ((float)m2 / (float)THROTTLE_LIMIT);
    m[2] = ((float)m3 / (float)THROTTLE_LIMIT);
    m[3] = ((float)m4 / (float)THROTTLE_LIMIT);

    esc_set(m);
#endif
}
