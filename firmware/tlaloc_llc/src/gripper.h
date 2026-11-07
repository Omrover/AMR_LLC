/*
 * Gripper: servo de posición (PWM 50 Hz) + ACS712 en serie con su alimentación.
 * C99 portable (pruebas en host); el HAL aplica el pulso y lee el ADC.
 *
 * El servo no tiene realimentación de posición: grip.pos es la posición
 * comandada (con límite de velocidad). El agarre se detecta por corriente:
 * mientras cierra, si |I| filtrada supera max_cur durante grasp_ms, se congela
 * la posición donde está (el servo sigue apretando con esa corriente) y se
 * marca grasped; la consigna vuelve al punto donde la corriente empezó a
 * subir. Cerrar hasta el tope sin superar max_cur NO es agarre.
 *
 * Al arrancar se mide el cero del ACS712 (servo quieto, zero_cal_ms) y hasta
 * entonces no se mueve.
 */
#ifndef TLALOC_GRIPPER_H
#define TLALOC_GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t pulse_open_us;     /* pulso con pos = 0 (abierto) */
	uint32_t pulse_closed_us;   /* pulso con pos = 1 (cerrado) */
	float speed;                /* recorrido completo por segundo (1/s) */
	float mv_per_a;             /* sensibilidad vista en el pin del ADC (con divisor) */
	float cur_tau_s;            /* constante de tiempo del filtro de corriente */
	uint32_t grasp_ms;          /* tiempo sobre max_cur para declarar agarre */
	float hard_limit_a;         /* sobrecorriente: abre un poco y marca falla */
	uint32_t hard_limit_ms;
	float back_off;             /* cuánto abre ante sobrecorriente (0..1) */
	uint32_t zero_cal_ms;
} grip_cfg_t;

typedef struct {
	grip_cfg_t cfg;
	float target;
	float pos;
	float max_cur;
	float contact_pos;
	float cur_a;                /* |I| filtrada */
	float zero_mv;
	bool zero_ready;
	bool grasped;
	bool overcurrent;           /* falla latcheada hasta el siguiente comando */
	bool adc_fault;
	uint32_t cal_ms;
	double cal_acc;
	uint32_t cal_n;
	uint32_t over_ms;
	uint32_t hard_ms;
} grip_t;

void grip_default_cfg(grip_cfg_t *cfg);
void grip_init(grip_t *g, const grip_cfg_t *cfg);
/* pos 0 (abierto) .. 1 (cerrado); max_cur en A. Abrir borra grasped. */
void grip_command(grip_t *g, float pos, float max_cur);
/* Un ciclo. adc_mv: salida del ACS712 en el pin del ADC; adc_ok = lectura válida.
 * Devuelve el pulso a aplicar en microsegundos. */
uint32_t grip_step(grip_t *g, uint32_t dt_ms, float adc_mv, bool adc_ok);
uint32_t grip_pulse_us(const grip_t *g);

#ifdef __cplusplus
}
#endif

#endif /* TLALOC_GRIPPER_H */
