#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    int32_t ref;
    float kp;
    float ki;
    float kd;
} pid_handler_t;

void pid_init(pid_handler_t hpid);
int pid_run(int32_t fb, int32_t t);

void pid_set_ref(int32_t new_ref);

#endif /* PID_H */
