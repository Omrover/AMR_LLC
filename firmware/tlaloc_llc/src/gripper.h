/*
 * Gripper: motorreductor DC (servo sin su electrónica) con un DRV8833 y un ACS712
 * en serie con el motor. C99 portable (pruebas en host); el HAL aplica la salida
 * al puente H (dos PWM + nSLEEP) y lee el ADC y nFAULT.
 *
 * Sin realimentación de posición (el potenciómetro del servo queda para después):
 * grip.pos se estima integrando el tiempo de movimiento (travel_ms = carrera
 * completa a move_duty) y se corrige en los topes, que se detectan por corriente.
 *
 *  - Abrir: gira hacia abrir hasta que la corriente pasa stall_a (tope) o se
 *    agota la carrera estimada más overtravel_ms. Luego suelta y duerme el driver.
 *  - Cerrar: gira hacia cerrar a move_duty. Si se traba (|I| > min(max_cur,
 *    stall_close_a) durante grasp_ms) antes de llegar a closed_pos, hay pieza: grasped y
 *    pasa a SOSTENER, un PI que ajusta el duty para mantener |I| = max_cur (fuerza
 *    de agarre constante, sin quemar el motor en bloqueo). Si se traba ya
 *    cerrado, no hay pieza: se detiene sin grasped.
 *  - Protecciones: |I| > hard_limit_a durante hard_limit_ms, o nFAULT del
 *    DRV8833 activo drv_fault_ms seguidos (se ignora drv_wake_ms tras despertar el
 *    driver): falla latcheada (overcurrent / driver_fault) hasta el siguiente
 *    comando; abre back_off_ms y se detiene.
 *  - Al arrancar la posición es desconocida (homed = false): la primera apertura
 *    empuja la carrera completa (travel_ms + overtravel_ms) o hasta el tope.
 *
 * Al arrancar se mide el cero del ACS712 (driver dormido, zero_cal_ms) y hasta
 * entonces no se mueve. Después, cada vez que el driver lleva zero_settle_ms dormido
 * (motor sin corriente) el cero se sigue con un filtro lento (zero_tau_s): corrige
 * un cero tomado con el sensor aún sin alimentar y la deriva térmica del ACS712.
 */
#ifndef TLALOC_GRIPPER_H
#define TLALOC_GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t travel_ms;         /* carrera completa a move_duty */
	uint32_t overtravel_ms;     /* empuje extra al abrir para asegurar el tope */
	float move_duty;            /* 0..1 al abrir y al acercarse a la pieza */
	float max_duty;             /* límite del PI al sostener */
	/* PI de corriente normalizado por la ganancia del motor sobre la pieza (k_est,
	 * A por unidad de duty, medida al contacto): el lazo responde igual con un
	 * motor de 0.2 A o de 2 A trabado. */
	float kp;                   /* adimensional */
	float ki;                   /* 1/s (~1/constante de tiempo) */
	float closed_pos;           /* trabarse más allá de esto = cerró en vacío */
	float stall_a;              /* corriente de motor trabado a move_duty: tope de abierto */
	float stall_close_a;        /* lo mismo al cerrar (contacto / tope de cerrado); el motor
	                             * puede consumir distinto en cada sentido */
	float mv_per_a;             /* sensibilidad vista en el pin del ADC (con divisor) */
	float cur_tau_s;            /* constante de tiempo del filtro de corriente */
	uint32_t grasp_ms;          /* tiempo sobre max_cur para declarar agarre/tope */
	float hard_limit_a;         /* sobrecorriente */
	uint32_t hard_limit_ms;
	uint32_t back_off_ms;       /* cuánto abre ante una falla */
	uint32_t start_ignore_ms;   /* pico de arranque: no cuenta como tope ni agarre */
	uint32_t zero_cal_ms;
	uint32_t zero_settle_ms;    /* dormido este tiempo antes de seguir el cero */
	uint32_t drv_fault_ms;      /* nFAULT sostenido este tiempo = falla */
	uint32_t drv_wake_ms;       /* nFAULT ignorado al despertar el driver */
	float zero_tau_s;           /* constante de tiempo del seguimiento del cero */
} grip_cfg_t;

typedef enum {
	GRIP_IDLE = 0,              /* driver dormido, motor suelto */
	GRIP_OPENING,
	GRIP_CLOSING,
	GRIP_HOLDING,               /* PI de corriente sobre la pieza */
	GRIP_BACK_OFF,              /* abriendo tras una falla */
} grip_state_t;

/* Salida al puente H. duty > 0 cierra, < 0 abre (|duty| <= 1). */
typedef struct {
	float duty;
	bool sleep;                 /* true: nSLEEP bajo (salidas en alta impedancia) */
} grip_out_t;

typedef struct {
	grip_cfg_t cfg;
	grip_state_t state;
	float target;
	float pos;                  /* estimada 0 (abierto) .. 1 (cerrado) */
	float max_cur;
	float cur_a;                /* |I| filtrada */
	float duty;                 /* última salida */
	float integ;                /* integrador del PI (duty) */
	float k_est;                /* A por unidad de duty sobre la pieza (0 = sin medir) */
	float zero_mv;
	bool zero_ready;
	bool grasped;
	bool overcurrent;           /* falla latcheada hasta el siguiente comando */
	bool driver_fault;          /* nFAULT del DRV8833, latcheada igual */
	bool homed;                 /* se conoce la posición (tope de abierto visto) */
	uint32_t nfault_ms;         /* tiempo seguido con nFAULT activo */
	bool adc_fault;
	uint32_t cal_ms;
	double cal_acc;
	uint32_t cal_n;
	uint32_t over_ms;           /* tiempo sobre max_cur / stall_a */
	uint32_t hard_ms;
	uint32_t timer_ms;          /* sobrecarrera / back-off */
	uint32_t run_ms;            /* desde el inicio del movimiento */
} grip_t;

void grip_default_cfg(grip_cfg_t *cfg);
void grip_init(grip_t *g, const grip_cfg_t *cfg);
/* pos 0 (abierto) .. 1 (cerrado); max_cur en A (corriente de agarre). Abrir
 * suelta la pieza; un comando nuevo borra las fallas latcheadas. */
void grip_command(grip_t *g, float pos, float max_cur);
/* Detiene el movimiento (e-stop). Si sostiene una pieza, la sigue sosteniendo. */
void grip_stop(grip_t *g);
/* Un ciclo. adc_mv: salida del ACS712 en el pin del ADC; adc_ok = lectura válida;
 * drv_fault = nFAULT activo. Devuelve la salida para el puente H. */
grip_out_t grip_step(grip_t *g, uint32_t dt_ms, float adc_mv, bool adc_ok, bool drv_fault);

#ifdef __cplusplus
}
#endif

#endif /* TLALOC_GRIPPER_H */
