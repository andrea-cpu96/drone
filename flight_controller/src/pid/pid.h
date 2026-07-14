#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    int32_t ref;
    float kp;
    float ki;
    float kd;
    float integral;
    int32_t e_prev;
    int32_t t_prev;
} pid_handler_t;

int pid_run(pid_handler_t *pid, int32_t fb, int32_t t);

void pid_set_ref(pid_handler_t *pid, int32_t new_ref);
void pid_init(pid_handler_t *pid, int32_t ref, float kp, float ki, float kd);

#endif // PID_H
