#include "pid.h"

#define SAMPLE_WINDOW 100
#define TIME_WINDOW 100

static int32_t ref = 0;
static float kp = 0.0;
static float ki = 0.0;
static float kd = 0.0;

static int32_t e_prev = 0;
static int32_t t_prev = 0;

static int32_t pid_p(int e);
static int32_t pid_i(int e, int dt_ms);
static int32_t pid_d(int e, int dt_ms);

/**
 * @brief PID init function
 *
 * @param hpid
 */
void pid_init(pid_handler_t hpid)
{
    pid_set_ref(hpid.ref);
    kp = hpid.kp;
    ki = hpid.ki;
    kd = hpid.kd;
}

/**
 * @brief PID run function
 *
 * @param fb
 * @param t
 * @return int32_t
 */
int pid_run(int32_t fb, int32_t t)
{
    int32_t control_out = 0;

    // Calculate the time difference
    int dt = (t - t_prev);

    // Evaluate the error
    int32_t e = (ref - fb);

    // Proportional component
    control_out = pid_p(e);

    // Integral component
    control_out += pid_i(e, dt);

    // Derivative component
    if(dt > 0)
        control_out += pid_d(e, dt);

    // Save the previous error
    e_prev = e;

    // Save the previous time
    t_prev = t;

    // Return the output controller
    return control_out;
}

/**
 * @brief Dynamic change of reference
 *
 * @param new_ref
 */
void pid_set_ref(int32_t new_ref)
{
    ref = new_ref;
}

/**
 * @brief Proportional component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_p(int e)
{
    return (kp * e);
}

/**
 * @brief Integral component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_i(int e, int dt_ms)
{
    static float integral = 0.0f;

    integral += ((float)e * (float)dt_ms / 1000.0f);

    return (ki * integral);
}

/**
 * @brief Derivative component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_d(int e, int dt_ms)
{
    if (dt_ms <= 0)
        return 0;

    float de = (float)(e - e_prev);
    float d = kd * (de * 1000.0f / (float)dt_ms);

    return (int32_t)d;
}