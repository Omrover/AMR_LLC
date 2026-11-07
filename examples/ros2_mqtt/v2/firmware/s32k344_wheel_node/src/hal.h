#ifndef LAB_HAL_H
#define LAB_HAL_H

#include <stdint.h>

/* Initialise encoder + motor outputs; outputs are left at zero. */
int hal_init(void);

/* Cumulative left-wheel angle in radians since boot. */
int hal_read_rotation(double *rotation_rad);

/* Decoded encoder ticks per wheel revolution (from devicetree). */
double hal_ticks_per_rev(void);

/* Apply signed duty [-1, 1] to both wheels via student_hbridge_pulses(). */
int hal_set_wheels(float left, float right);

#endif
