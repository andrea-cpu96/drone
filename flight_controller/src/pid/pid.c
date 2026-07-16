#include "pid.h"

#include "log.h"

static int32_t pid_p(pid_handler_t *pid, float e);
static int32_t pid_i(pid_handler_t *pid, float e, int dt_ms);
static int32_t pid_d(pid_handler_t *pid, float e, int dt_ms);

/**
 * @brief PID initialization
 *
 * @param hpid
 */
void pid_init(pid_handler_t *pid, float ref, float kp, float ki, float kd)
{
    pid->ref = ref;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->e_prev = 0;
    pid->t_prev = 0;

    log_print("PID initialized (ref=%.1f kp=%.1f ki=%.1f kd=%.1f)\n", (double)ref, (double)kp, (double)ki, (double)kd);
}

/**
 * @brief PID run function
 *
 * @param fb
 * @param t
 * @return int32_t
 */
int pid_run(pid_handler_t *pid, float fb, int32_t t)
{
    int32_t control_out = 0;

    // Calculate the time difference
    int dt = (t - pid->t_prev);

    // Evaluate the error
    float e = pid->ref - fb;

    // Proportional component
    control_out = pid_p(pid, e);

    // Integral component
    control_out += pid_i(pid, e, dt);

    // Derivative component
    if (dt > 0)
        control_out += pid_d(pid, e, dt);

    // Save the previous error
    pid->e_prev = e;

    // Save the previous time
    pid->t_prev = t;

    // Return the output controller
    return control_out;
}

/**
 * @brief Dynamic change of reference
 *
 * @param new_ref
 */
void pid_set_ref(pid_handler_t *pid, float new_ref)
{
    pid->ref = new_ref;
}

/**
 * @brief Proportional component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_p(pid_handler_t *pid, float e)
{
    return (int32_t)(pid->kp * e);
}

/**
 * @brief Integral component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_i(pid_handler_t *pid, float e, int dt_ms)
{
    pid->integral += e * ((float)dt_ms / 1000.0f);

    return (int32_t)(pid->ki * pid->integral);
}

/**
 * @brief Derivative component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_d(pid_handler_t *pid, float e, int dt_ms)
{
    if (dt_ms <= 0)
        return 0;

    float de = e - pid->e_prev;
    float d = pid->kd * (de * 1000.0f / (float)dt_ms);

    return (int32_t)d;
}
