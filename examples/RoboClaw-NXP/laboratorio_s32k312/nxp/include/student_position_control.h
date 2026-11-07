/* Lazo externo P/PD de posición. La salida es una velocidad en conteos/s.
 * Kp tiene unidades 1/s; Kd multiplica la velocidad medida y es adimensional.
 * El PID de velocidad convierte esa referencia en duty para RoboClaw.
 */
#ifndef STUDENT_POSITION_CONTROL_H
#define STUDENT_POSITION_CONTROL_H
#include <stdint.h>
#include "student_speed_pid.h"

typedef struct {
    float kp, kd;
    int32_t tolerance;
    unsigned int settled_steps, reached;
} StudentPositionControl;

static inline void student_position_reset(StudentPositionControl *control)
{
    control->settled_steps = 0U; control->reached = 0U;
}

static inline float student_position_step(StudentPositionControl *control,
    int32_t target, int32_t position, float speed, float minimum, float maximum)
{
    int64_t error = (int64_t)target - position;
    int64_t magnitude = error < 0 ? -error : error;
    if (control->reached && magnitude <= (int64_t)control->tolerance * 2 &&
        speed >= -20.0f && speed <= 20.0f) return 0.0f;
    if (control->reached) student_position_reset(control);
    if (magnitude <= control->tolerance && speed >= -10.0f && speed <= 10.0f)
        ++control->settled_steps;
    else control->settled_steps = 0U;
    if (control->settled_steps >= 5U) {
        control->reached = 1U; return 0.0f;
    }
    /* Cerca del destino pedimos velocidad cero para frenar, no para desarmar. */
    if (magnitude <= control->tolerance) return 0.0f;
    return pid_clamp(control->kp * (float)error - control->kd * speed, minimum, maximum);
}

static inline unsigned int student_position_selftest(void)
{
    StudentPositionControl p = {.kp = 0.5f, .kd = 0.0f, .tolerance = 10};
    unsigned int errors = 0U, i;
    if (student_position_step(&p, 1000, 0, 0, -300, 300) != 300) errors |= 1U;
    if (student_position_step(&p, -1000, 0, 0, -300, 300) != -300) errors |= 2U;
    p.kd = 0.5f;
    if (student_position_step(&p, 100, 0, 20, -300, 300) != 40) errors |= 4U;
    for (i = 0U; i < 5U; ++i) (void)student_position_step(&p, 100, 95, 0, -300, 300);
    if (!p.reached) errors |= 8U;
    if (student_position_step(&p, 100, 81, 0, -300, 300) != 0 || !p.reached) errors |= 16U;
    if (student_position_step(&p, 100, 70, 0, -300, 300) <= 0 || p.reached) errors |= 32U;
    student_position_reset(&p);
    for (i = 0U; i < 5U; ++i) (void)student_position_step(&p, 100, 100, 50, -300, 300);
    if (p.reached) errors |= 64U;
    if (student_position_step(&p, INT32_MAX, INT32_MIN, 0, -300, 300) != 300) errors |= 128U;
    return errors;
}
#endif
