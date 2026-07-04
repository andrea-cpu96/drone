#include "esc.h"

#include <stdio.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#define PWM_PERIOD_NS 1000000
#define PWM_PULSE_NS 500000
#define CH_NUM_MAX 4

#ifndef CONFIG_SIMULATION_MODE
const struct device *esc_pwm = DEVICE_DT_GET(DT_ALIAS(esc_controller));
int esc_ch_num;
#endif  // CONFIG_SIMULATION_MODE

esc_status esc_init(int n_ch)
{
#ifndef CONFIG_SIMULATION_MODE
    int pwm_ok;

    esc_ch_num = n_ch;

    if (esc_ch_num > CH_NUM_MAX)
    {
        return ESC_ERR;
    }

    if (!device_is_ready(esc_pwm))
    {
        printf("ESC device not ready\n");
        return ESC_ERR;
    }

    for (int i = 1; i <= esc_ch_num; i++)
    {
        pwm_ok = pwm_set(esc_pwm, i, PWM_PERIOD_NS, PWM_PULSE_NS, 0);
        if (pwm_ok < 0)
        {
            printk("failed to set idle (%d)\n", pwm_ok);
            return ESC_ERR;
        }
    }
    return ESC_OK;
#else
    return ESC_OK;
#endif  // CONFIG_SIMULATION_MODE
}

esc_status esc_set(float *m)
{
#ifndef CONFIG_SIMULATION_MODE
    float speed_perc = 0.0f;

    for (int i = 0; i < esc_ch_num; i++)
    {
        if ((m[i] < 0.0f) || (m[i] > 1.0f))
        {
            return ESC_ERR;
        }
    }

    for (int i = 0; i < esc_ch_num; i++)
    {
        speed_perc = (m[i] * (float)PWM_PERIOD_NS);
        pwm_set(esc_pwm, i+1, PWM_PERIOD_NS, (int)speed_perc, 0);
    }

    return ESC_OK;
#else
    return ESC_OK;
#endif  // CONFIG_SIMULATION_MODE
}