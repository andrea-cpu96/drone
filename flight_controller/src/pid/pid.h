#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float ref;
    float kp;
    float ki;
    float kd;
    float integral;
    float e_prev;
    int32_t t_prev;
} pid_handler_t;

int pid_run(pid_handler_t *pid, float fb, int32_t t);
void pid_set_ref(pid_handler_t *pid, float new_ref);
void pid_init(pid_handler_t *pid, float ref, float kp, float ki, float kd);

#endif  // PID_H
