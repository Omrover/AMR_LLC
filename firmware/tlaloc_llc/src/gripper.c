#include "gripper.h"

#include <math.h>
#include <string.h>

#define POS_EPS 1e-3f

void grip_default_cfg(grip_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->travel_ms = 1500;
	cfg->overtravel_ms = 300;
	cfg->move_duty = 0.6f;
	cfg->max_duty = 0.9f;
	/* PI de corriente: con el motor trabado I ~ duty * I_bloqueo; kp/ki chicos
	 * porque el lazo corre a 50 Hz y el ACS712 es ruidoso. */
	cfg->kp = 0.15f;
	cfg->ki = 1.5f;
	cfg->closed_pos = 0.95f;
	cfg->stall_a = 0.8f;
	/* ACS712-05B: 185 mV/A a 5 V. Con un divisor 2/3 (10k/20k) hacia el ADC de
	 * 3.3 V se ve 123.3 mV/A; con 1/2 serían 92.5 (TLALOC_ACS712_MV_PER_A_X10). */
	cfg->mv_per_a = 123.3f;
	cfg->cur_tau_s = 0.03f;
	cfg->grasp_ms = 80;
	cfg->hard_limit_a = 2.0f;      /* DRV8833: 1.5 A RMS / 2 A pico por canal */
	cfg->hard_limit_ms = 300;
	cfg->back_off_ms = 200;
	cfg->start_ignore_ms = 150;
	cfg->zero_cal_ms = 500;
	cfg->zero_settle_ms = 200;
	cfg->zero_tau_s = 0.5f;
}

void grip_init(grip_t *g, const grip_cfg_t *cfg)
{
	memset(g, 0, sizeof(*g));
	g->cfg = *cfg;
	g->max_cur = 1.0f;
	g->state = GRIP_IDLE;
}

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static void start(grip_t *g, grip_state_t st)
{
	g->state = st;
	g->over_ms = 0;
	g->timer_ms = 0;
	g->run_ms = 0;
}

void grip_command(grip_t *g, float pos, float max_cur)
{
	pos = clampf(pos, 0.0f, 1.0f);
	g->target = pos;
	g->max_cur = max_cur;
	g->overcurrent = false;
	g->driver_fault = false;
	g->hard_ms = 0;
	if (pos <= POS_EPS || pos < g->pos - POS_EPS) {
		/* abrir (pos 0 siempre busca el tope para corregir la estimación) */
		g->grasped = false;
		start(g, GRIP_OPENING);
	} else if (g->grasped) {
		/* ya sostiene: el comando solo cambia la fuerza (max_cur) */
		if (g->state != GRIP_HOLDING) {
			start(g, GRIP_HOLDING);
			g->integ = g->cfg.move_duty;
		}
	} else if (pos > g->pos + POS_EPS || pos >= 1.0f) {
		start(g, GRIP_CLOSING);
	} else {
		start(g, GRIP_IDLE);
	}
}

void grip_stop(grip_t *g)
{
	if (g->state != GRIP_HOLDING) {
		start(g, GRIP_IDLE);
	}
}

static void move(grip_t *g, float dir, float dt)
{
	float travel = (float)g->cfg.travel_ms * 1e-3f;

	if (travel > 0.0f) {
		g->pos = clampf(g->pos + dir * dt / travel, 0.0f, 1.0f);
	}
}

/* |I| sobre limit durante grasp_ms (fuera del pico de arranque). */
static bool sustained(grip_t *g, bool adc_ok, float limit, uint32_t dt_ms)
{
	if (adc_ok && g->run_ms > g->cfg.start_ignore_ms && g->cur_a > limit) {
		g->over_ms += dt_ms;
		return g->over_ms >= g->cfg.grasp_ms;
	}
	g->over_ms = 0;
	return false;
}

grip_out_t grip_step(grip_t *g, uint32_t dt_ms, float adc_mv, bool adc_ok, bool drv_fault)
{
	float dt = (float)dt_ms * 1e-3f;
	grip_out_t out = {0.0f, true};

	g->adc_fault = !adc_ok;
	if (!g->zero_ready) {
		/* driver dormido: el ACS712 ve 0 A */
		if (adc_ok) {
			g->cal_acc += (double)adc_mv;
			g->cal_n++;
		}
		g->cal_ms += dt_ms;
		if (g->cal_ms >= g->cfg.zero_cal_ms && g->cal_n > 0U) {
			g->zero_mv = (float)(g->cal_acc / (double)g->cal_n);
			g->zero_ready = true;
		}
		g->duty = 0.0f;
		return out;
	}

	if (adc_ok) {
		float i = fabsf((adc_mv - g->zero_mv) / g->cfg.mv_per_a);
		float a = dt / (g->cfg.cur_tau_s + dt);

		g->cur_a += a * (i - g->cur_a);
	}
	g->run_ms += dt_ms;

	/* nFAULT (sobrecorriente o temperatura del DRV8833): se detiene. */
	if (drv_fault && g->state != GRIP_IDLE) {
		g->driver_fault = true;
		g->grasped = false;
		start(g, GRIP_IDLE);
	}
	/* Sobrecorriente sostenida: falla y abre un poco. */
	if (adc_ok && g->state != GRIP_IDLE && g->cur_a > g->cfg.hard_limit_a) {
		g->hard_ms += dt_ms;
		if (g->hard_ms >= g->cfg.hard_limit_ms) {
			g->overcurrent = true;
			g->grasped = false;
			g->hard_ms = 0;
			start(g, GRIP_BACK_OFF);
		}
	} else {
		g->hard_ms = 0;
	}

	switch (g->state) {
	case GRIP_OPENING:
		g->duty = -g->cfg.move_duty;
		move(g, -1.0f, dt);
		if (sustained(g, adc_ok, g->cfg.stall_a, dt_ms)) {
			g->pos = 0.0f;       /* tope de abierto: corrige la estimación */
			start(g, GRIP_IDLE);
		} else if (g->target > POS_EPS && g->pos <= g->target) {
			start(g, GRIP_IDLE); /* apertura parcial */
		} else if (g->pos <= 0.0f) {
			g->timer_ms += dt_ms;
			if (g->timer_ms >= g->cfg.overtravel_ms) {
				start(g, GRIP_IDLE);
			}
		}
		break;
	case GRIP_CLOSING:
		g->duty = g->cfg.move_duty;
		move(g, 1.0f, dt);
		/* contacto = el motor se traba (stall_a), o antes si max_cur es menor */
		if (sustained(g, adc_ok, fminf(g->max_cur, g->cfg.stall_a), dt_ms)) {
			if (g->pos < g->cfg.closed_pos) {
				g->grasped = true;   /* se trabó antes del tope: hay pieza */
				g->integ = g->cfg.move_duty;
				start(g, GRIP_HOLDING);
			} else {
				g->pos = 1.0f;       /* cerró en vacío */
				start(g, GRIP_IDLE);
			}
		} else if (g->target < 1.0f && g->pos >= g->target) {
			start(g, GRIP_IDLE);
		} else if (g->pos >= 1.0f) {
			g->timer_ms += dt_ms;
			if (g->timer_ms >= g->cfg.overtravel_ms) {
				start(g, GRIP_IDLE);
			}
		}
		break;
	case GRIP_HOLDING:
		if (adc_ok) {
			float e = g->max_cur - g->cur_a;

			g->integ = clampf(g->integ + g->cfg.ki * e * dt, 0.0f, g->cfg.max_duty);
			g->duty = clampf(g->cfg.kp * e + g->integ, 0.0f, g->cfg.max_duty);
		} else {
			g->duty = g->integ;  /* sin medición: mantiene el último duty */
		}
		break;
	case GRIP_BACK_OFF:
		g->duty = -g->cfg.move_duty;
		move(g, -1.0f, dt);
		g->timer_ms += dt_ms;
		if (g->timer_ms >= g->cfg.back_off_ms) {
			start(g, GRIP_IDLE);
		}
		break;
	case GRIP_IDLE:
	default:
		g->duty = 0.0f;
		/* Driver dormido = I real 0: el cero sigue a la lectura (ver gripper.h). */
		g->timer_ms += dt_ms;
		if (adc_ok && g->timer_ms >= g->cfg.zero_settle_ms) {
			float a = dt / (g->cfg.zero_tau_s + dt);

			g->zero_mv += a * (adc_mv - g->zero_mv);
			g->cur_a = fabsf((adc_mv - g->zero_mv) / g->cfg.mv_per_a);
		}
		break;
	}

	out.duty = g->state == GRIP_IDLE ? 0.0f : g->duty;
	out.sleep = g->state == GRIP_IDLE;
	return out;
}
