#ifndef STUDENT_SPEED_PID_SELFTEST_H
#define STUDENT_SPEED_PID_SELFTEST_H
#include "student_speed_pid.h"

/* Probamos las cuentas al arrancar, sin enviar nada al motor. 0 = correcto.
 * También podemos ejecutar estas pruebas sin los headers de NXP.
 */
static inline unsigned int student_pid_selftest(void)
{
    StudentSpeedPid p = {.kp = 10.0f, .ki = 10.0f, .kd = 0.0f, .duty_limit = 15.0f};
    unsigned int errors = 0U, i;
    float previous = 0.0f, output;
    student_pid_reset(&p, 0.0f);
    if (p.integral != 0.0f || p.output != 0.0f || p.reference != 0.0f) errors |= 1U;
    for (i = 0U; i < 200U; ++i) {
        output = student_pid_step(&p, 10000.0f, 0.0f, 0.05f);
        if (output < 0.0f || output > 15.001f || output - previous > 2.501f) errors |= 2U;
        previous = output;
    }
    if (p.integral != 0.0f || p.output < 14.99f) errors |= 4U;
    student_pid_reset(&p, 0.0f);
    previous = 0.0f;
    for (i = 0U; i < 200U; ++i) {
        output = student_pid_step(&p, -10000.0f, 0.0f, 0.05f);
        if (output > 0.0f || output < -15.001f || previous - output > 2.501f) errors |= 8U;
        previous = output;
    }
    if (p.integral != 0.0f || p.output > -14.99f) errors |= 16U;
    p.kp = 0.02f; p.ki = 0.05f; p.kd = 0.1f;
    student_pid_reset(&p, 0.0f);
    (void)student_pid_step(&p, 100.0f, 0.0f, 0.05f);
    if (p.reference != 25.0f || p.derivative != 0.0f || p.integral <= 0.0f) errors |= 32U;
    (void)student_pid_step(&p, 100.0f, 50.0f, 0.05f);
    if (p.derivative <= 0.0f) errors |= 64U;
    student_pid_reset(&p, 50.0f);
    if (p.integral != 0.0f || p.derivative != 0.0f || p.output != 0.0f ||
        p.previous_speed != 50.0f || p.reference != 0.0f) errors |= 128U;
    p.reference = 100.0f; p.output = 4.0f; p.derivative = 2.0f;
    student_pid_tune(&p, 0.03f, 0.05f, 0.1f, 50.0f);
    output = p.kp * 50.0f + p.integral - p.kd * p.derivative;
    if (output < 3.999f || output > 4.001f || p.reference != 100.0f ||
        p.derivative != 2.0f || p.output != 4.0f) errors |= 256U;
    student_pid_tune(&p, 0.03f, 0.0f, 0.0f, 50.0f);
    if (p.integral != 0.0f) errors |= 512U;
    previous = p.output;
    output = student_pid_step(&p, 100.0f, 50.0f, 0.05f);
    if (output - previous > 2.501f || previous - output > 2.501f) errors |= 1024U;
    previous = p.output;
    student_pid_set_limit(&p, 40.0f);
    if (p.output != previous || p.duty_limit != 40.0f) errors |= 2048U;
    p.kp = 10.0f; p.ki = 10.0f; p.kd = 0.0f;
    student_pid_reset(&p, 0.0f);
    previous = 0.0f;
    for (i = 0U; i < 200U; ++i) {
        output = student_pid_step(&p, 10000.0f, 0.0f, 0.05f);
        if (output > 40.001f || output - previous > 2.501f) errors |= 4096U;
        previous = output;
    }
    if (p.output < 39.99f || p.integral != 0.0f) errors |= 4096U;
    p.integral = 20.0f;
    student_pid_set_limit(&p, 5.0f);
    if (p.integral != 5.0f || p.output != 5.0f) errors |= 8192U;
    student_pid_tune(&p, 0.0f, 0.1f, 0.0f, 0.0f);
    if (p.integral > 5.0f || p.integral < -5.0f) errors |= 8192U;
    student_pid_set_limit(&p, 40.0f);
    p.kp = 10.0f; p.ki = 10.0f;
    student_pid_reset(&p, 0.0f);
    previous = 0.0f;
    for (i = 0U; i < 200U; ++i) {
        output = student_pid_step(&p, -10000.0f, 0.0f, 0.05f);
        if (output < -40.001f || previous - output > 2.501f) errors |= 16384U;
        previous = output;
    }
    if (p.output > -39.99f || p.integral != 0.0f) errors |= 16384U;
    student_pid_set_limit(&p, 0.0f);
    output = student_pid_step(&p, 10000.0f, 0.0f, 0.05f);
    if (output != 0.0f || p.integral != 0.0f) errors |= 32768U;
    student_pid_reset(&p, 0.0f);
    if (p.duty_limit != 0.0f) errors |= 32768U;
    return errors;
}
#endif
