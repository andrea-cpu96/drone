#include <stdio.h>

#include <zephyr/kernel.h>

#include "esc.h"
#include "pid.h"
#include "sensors.h"

#define REFERENCE_ALTITUDE 100

#define CONTROL_THREAD_PERIOD_MS 10
#define SENSORS_THREAD_PERIOD_MS 100

#define LIFTOFF_THROTTLE 2092
#define THROTTLE_LIMIT 4000
#define MOTOR_NUM 4

// Controller thread configuration
#define PID_THREAD_PRIO 1
K_THREAD_STACK_DEFINE(controller_stack, 2048);
static struct k_thread controller_tid;
// Sensors thread configuration
#define SENSORS_THREAD_PRIO 5
K_THREAD_STACK_DEFINE(sensors_stack, 2048);
static struct k_thread sensors_tid;

pid_handler_t throttle_pid;
static int motor[MOTOR_NUM] = {0, 0, 0, 0};
#if defined(CONFIG_ESC_VERIFY_MODE)
static int motor_command_started = 0;
static uint32_t motor_command_start_ms = 0;
static uint32_t motor_ramp_last_update_ms = 0;
static float motor_throttle = 0.0f;
#endif

static void send_motor_command(int m1, int m2, int m3, int m4);
static void sensors_thread(void *a, void *b, void *c);
#if defined(CONFIG_ESC_VERIFY_MODE)
static float esc_verify_throttle(void);
#endif

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
         * Read the latest altitude published by the sensors thread. The value is
         * a word-sized scalar written with a single store, so the read is atomic
         * and always returns a complete value.
         */
        altitude_feedback = sensors_read_altitude();

        control = pid_run(&throttle_pid, altitude_feedback, time);

        time += CONTROL_THREAD_PERIOD_MS;

        /*
         * These kind of prints must be moved in a dedicated logging thread, to avoid
         * blocking the control loop.
         */
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
    }
}

static void sensors_thread(void *a, void *b, void *c)
{
    while (1)
    {
        sensors_altitude_process();
        // Call the process function of every new sensor here.

        k_msleep(SENSORS_THREAD_PERIOD_MS);
    }
}

int main(void)
{
    printf("Controller thread starting...\n");

    pid_init(&throttle_pid, REFERENCE_ALTITUDE, 21, 5, 8);
    esc_init(MOTOR_NUM);

    // Initialize sensors (barometer, etc.) and perform an initial read
    sensors_init();

    k_thread_create(&controller_tid, controller_stack,
                    K_THREAD_STACK_SIZEOF(controller_stack), controller_thread, NULL,
                    NULL, NULL, PID_THREAD_PRIO, 0, K_NO_WAIT);

    k_thread_create(&sensors_tid, sensors_stack,
                    K_THREAD_STACK_SIZEOF(sensors_stack), sensors_thread, NULL,
                    NULL, NULL, SENSORS_THREAD_PRIO, 0, K_NO_WAIT);

    return 0;
}

/**
 * @brief Send motor command to the console
 *
 * @param m1
 * @param m2
 * @param m3
 * @param m4
 */
#if defined(CONFIG_ESC_VERIFY_MODE)
static float esc_verify_throttle(void)
{
    float throttle = 0.0f;
    uint32_t now_ms = k_uptime_get_32();

    if (!motor_command_started)
    {
        motor_command_start_ms = now_ms;
        motor_ramp_last_update_ms = now_ms;
        motor_command_started = 1;
    }

    if ((now_ms - motor_command_start_ms) >= CONFIG_ESC_VERIFY_DELAY_MS)
    {
        if ((now_ms - motor_ramp_last_update_ms) >= CONFIG_ESC_VERIFY_INTERVAL_MS)
        {
            if (motor_throttle < ((float)CONFIG_ESC_VERIFY_START_THROTTLE_PROMILLE / 1000.0f))
            {
                motor_throttle = ((float)CONFIG_ESC_VERIFY_START_THROTTLE_PROMILLE / 1000.0f);
            }
            else
            {
                motor_throttle += ((float)CONFIG_ESC_VERIFY_STEP_PROMILLE / 1000.0f);
            }

            if (motor_throttle > ((float)CONFIG_ESC_VERIFY_MAX_THROTTLE_PROMILLE / 1000.0f))
            {
                motor_throttle = ((float)CONFIG_ESC_VERIFY_MAX_THROTTLE_PROMILLE / 1000.0f);
            }

            motor_ramp_last_update_ms = now_ms;
        }
    }

    throttle = motor_throttle;

    return throttle;
}
#endif

static void send_motor_command(int m1, int m2, int m3, int m4)
{
#ifdef CONFIG_SIMULATION_MODE
    printf("%d.%03d %d.%03d %d.%03d %d.%03d\n", m1 / 1000, m1 % 1000, m2 / 1000,
           m2 % 1000, m3 / 1000, m3 % 1000, m4 / 1000, m4 % 1000);
           
    fflush(stdout);
#else
    float m[MOTOR_NUM] = {0.0f};

#    if defined(CONFIG_ESC_VERIFY_MODE)
    float throttle = esc_verify_throttle();

    m[0] = throttle;
    m[1] = throttle;
    m[2] = throttle;
    m[3] = throttle;
#    else
    float throttle = ((float)m1 / (float)THROTTLE_LIMIT);

    m[0] = throttle;
    m[1] = ((float)m2 / (float)THROTTLE_LIMIT);
    m[2] = ((float)m3 / (float)THROTTLE_LIMIT);
    m[3] = ((float)m4 / (float)THROTTLE_LIMIT);
#    endif

    esc_set(m);
#endif
}
