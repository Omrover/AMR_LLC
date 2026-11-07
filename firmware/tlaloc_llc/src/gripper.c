#include "gripper.h"

#include <math.h>
#include <string.h>

void grip_default_cfg(grip_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->pulse_open_us = 1000;   /* VERIFICAR con el servo MOT-135 */
	cfg->pulse_closed_us = 2000;
	cfg->speed = 1.0f;
	/* ACS712-05B: 185 mV/A a 5 V. Con un divisor 2/3 hacia el ADC de 3.3 V se ve
	 * 123.3 mV/A. VERIFICAR modelo y divisor. */
	cfg->mv_per_a = 123.3f;
	cfg->cur_tau_s = 0.03f;
	cfg->grasp_ms = 80;
	cfg->hard_limit_a = 3.0f;
	cfg->hard_limit_ms = 300;
	cfg->back_off = 0.1f;
	cfg->zero_cal_ms = 500;
}

void grip_init(grip_t *g, const grip_cfg_t *cfg)
{
	memset(g, 0, sizeof(*g));
	g->cfg = *cfg;
	g->max_cur = 1.0f;
}

static float clamp01(float v)
{
	return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

void grip_command(grip_t *g, float pos, float max_cur)
{
	pos = clamp01(pos);
	if (pos < g->pos) {
		g->grasped = false;   /* abrir suelta la pieza */
	}
	g->target = pos;
	g->max_cur = max_cur;
	g->overcurrent = false;
	g->over_ms = 0;       /* el pico de arranque del nuevo movimiento no cuenta */
	g->hard_ms = 0;
}

uint32_t grip_pulse_us(const grip_t *g)
{
	float span = (float)g->cfg.pulse_closed_us - (float)g->cfg.pulse_open_us;

	return (uint32_t)lroundf((float)g->cfg.pulse_open_us + g->pos * span);
}

uint32_t grip_step(grip_t *g, uint32_t dt_ms, float adc_mv, bool adc_ok)
{
	float dt = (float)dt_ms * 1e-3f;
	float step = g->cfg.speed * dt;

	g->adc_fault = !adc_ok;
	if (!g->zero_ready) {
		if (adc_ok) {
			g->cal_acc += adc_mv;
			g->cal_n++;
		}
		g->cal_ms += dt_ms;
		if (g->cal_ms >= g->cfg.zero_cal_ms && g->cal_n > 0U) {
			g->zero_mv = (float)(g->cal_acc / (double)g->cal_n);
			g->zero_ready = true;
		}
		return grip_pulse_us(g);   /* quieto mientras calibra */
	}

	if (adc_ok) {
		float i = fabsf((adc_mv - g->zero_mv) / g->cfg.mv_per_a);
		float a = dt / (g->cfg.cur_tau_s + dt);

		g->cur_a += a * (i - g->cur_a);
	}

	/* Sobrecorriente sostenida: marca falla y abre back_off; si sigue alta otro
	 * hard_limit_ms, vuelve a abrir (protege el servo de quedarse en bloqueo). */
	if (adc_ok && g->cur_a > g->cfg.hard_limit_a) {
		g->hard_ms += dt_ms;
		if (g->hard_ms >= g->cfg.hard_limit_ms) {
			g->overcurrent = true;
			g->grasped = false;
			g->target = clamp01(g->pos - g->cfg.back_off);
			g->hard_ms = 0;
		}
	} else {
		g->hard_ms = 0;
	}

	if (g->target > g->pos && !g->grasped) {
		/* cerrando: ¿tocó la pieza? (sin ADC no hay detección: no se asume agarre) */
		if (adc_ok && g->cur_a > g->max_cur) {
			if (g->over_ms == 0U) {
				g->contact_pos = g->pos;   /* donde empezó a subir la corriente */
			}
			g->over_ms += dt_ms;
			if (g->over_ms >= g->cfg.grasp_ms) {
				/* vuelve al punto de contacto: aprieta con la corriente
				 * de contacto, no con la del exceso de recorrido */
				g->grasped = true;
				g->target = g->contact_pos;
			}
		} else {
			g->over_ms = 0;
		}
	}

	if (g->pos < g->target) {
		g->pos = (g->target - g->pos) < step ? g->target : g->pos + step;
	} else if (g->pos > g->target) {
		g->pos = (g->pos - g->target) < step ? g->target : g->pos - step;
	}
	return grip_pulse_us(g);
}
