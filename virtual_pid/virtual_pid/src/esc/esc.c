#include "esc.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

const struct device *esc_pwm = DEVICE_DT_GET(DT_ALIAS(esc_controller));

void esc_init(void)
{
    int pwm_ok;

    if (!device_is_ready(esc_pwm))
    {
        printf("ESC device not ready\n");
        return;
    }

    pwm_ok = pwm_set(esc_pwm, 1, 1000000, 500000, 0);

	if (pwm_ok < 0) {
		printk("failed to set idle (%d)\n", pwm_ok);    
        return;
	}
}