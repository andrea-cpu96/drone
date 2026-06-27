#include "pid.h"

#define SAMPLE_WINDOW 100
#define TIME_WINDOW 100

static int32_t ref = 0;
static float kp = 0.0;
static float ki = 0.0;
static float kd = 0.0;

static int32_t sample[SAMPLE_WINDOW] = {0};
static int32_t time[TIME_WINDOW] = {0};

static int32_t pid_p(int e);
static int32_t pid_i(int e);
static int32_t pid_d(int e);

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
    static int32_t sample_i = 0;
    static int32_t time_i = 0;

    int32_t control_out = 0;

    // Save Feedback sample
    sample[sample_i] = fb;
    sample_i++;

    // Save time sample
    time[time_i] = t;
    time_i++;

    // Evaluate the error
    int32_t e = (ref - fb);

    // Proportional component
    control_out = pid_p(e);

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
static int32_t pid_i(int e)
{
    return (ki * e);
}

/**
 * @brief Derivative component of the PID
 *
 * @param e
 * @return int32_t
 */
static int32_t pid_d(int e)
{
    return (kd * e);
}