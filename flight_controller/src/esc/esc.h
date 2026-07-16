#include <stdint.h>

typedef enum
{
    ESC_OK = 0,
    ESC_ERR,
} esc_status;

esc_status esc_init(int n_ch);
esc_status esc_set(float *m);
