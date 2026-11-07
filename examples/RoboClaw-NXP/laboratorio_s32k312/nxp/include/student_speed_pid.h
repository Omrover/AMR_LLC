/* Este PID se calcula en la NXP; RoboClaw solo recibe el duty.
 * Referencia y medición: conteos/s. Salida e integral: porcentaje con signo.
 * Kp: %/(conteos/s), Ki: %/conteo, Kd: %/(conteos/s²).
 * Antes de entrar aquí validamos las ganancias y 0 < dt < 0.1 segundos.
 */
#ifndef STUDENT_SPEED_PID_H
#define STUDENT_SPEED_PID_H

typedef struct {
    float kp, ki, kd;
    float integral, derivative, previous_speed;
    float reference, output;
    float duty_limit;
} StudentSpeedPid;

static inline float pid_clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static inline void student_pid_reset(StudentSpeedPid *pid, float speed)
{
    pid->integral = 0.0f; pid->derivative = 0.0f;
    pid->previous_speed = speed; pid->reference = 0.0f; pid->output = 0.0f;
}

/* Subir el máximo no cambia la salida actual. Al bajarlo recortamos salida e
 * integral inmediatamente; el límite tiene prioridad sobre la rampa de bajada.
 */
static inline void student_pid_set_limit(StudentSpeedPid *pid, float limit)
{
    pid->duty_limit = pid_clamp(limit, 0.0f, 100.0f);
    pid->integral = pid_clamp(pid->integral, -pid->duty_limit, pid->duty_limit);
    pid->output = pid_clamp(pid->output, -pid->duty_limit, pid->duty_limit);
}

static inline float student_pid_step(StudentSpeedPid *pid, float target,
                                     float speed, float dt)
{
    float error, p, d, candidate, low, high, unconstrained;
    /* La referencia sube o baja con una rampa de 500 conteos/s². */
    pid->reference += pid_clamp(target - pid->reference, -500.0f * dt, 500.0f * dt);
    error = pid->reference - speed;
    p = pid->kp * error;
    /* Derivamos la medición, no la consigna. Filtro con tau de 100 ms. */
    pid->derivative += (dt / (0.1f + dt)) *
        ((speed - pid->previous_speed) / dt - pid->derivative);
    pid->previous_speed = speed;
    d = -pid->kd * pid->derivative;
    candidate = pid_clamp(pid->integral + pid->ki * error * dt,
                         -pid->duty_limit, pid->duty_limit);
    /* Limitamos el duty al máximo configurado y su cambio a 50 puntos porcentuales/s.
     * No seguimos acumulando integral si empuja hacia una saturación.
     */
    low = pid_clamp(pid->output - 50.0f * dt, -pid->duty_limit, pid->duty_limit);
    high = pid_clamp(pid->output + 50.0f * dt, -pid->duty_limit, pid->duty_limit);
    unconstrained = p + candidate + d;
    if (!((unconstrained > high && error > 0.0f) ||
          (unconstrained < low && error < 0.0f)))
        pid->integral = candidate;
    pid->output = pid_clamp(p + pid->integral + d, low, high);
    return pid->output;
}

/* Cambiamos las ganancias sin reiniciar la referencia ni el filtro.
 * Si hay integral, la reajustamos para conservar la salida anterior hasta
 * donde permite su límite. Con Ki=0 dejamos un P/PD real, sin integral acumulada.
 * La rampa de duty sigue limitando cualquier cambio que no se pueda compensar.
 */
static inline void student_pid_tune(StudentSpeedPid *pid, float kp, float ki,
                                    float kd, float speed)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = ki > 0.0f ? pid_clamp(pid->output - kp *
        (pid->reference - speed) + kd * pid->derivative,
        -pid->duty_limit, pid->duty_limit) : 0.0f;
}
#endif
