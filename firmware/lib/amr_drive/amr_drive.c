#include "amr_drive.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F 6.28318530718f

void amr_drive_default_cfg(amr_drive_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	/* Geometría medida (C:/Users/Ivani/routing, nov 2026): ruedas mecanum de
	 * 6" rectas, rodillos a 45 deg en X vistos desde arriba, en
	 * (+-Lx, +-Ly) = (+-0.1583, +-0.1734) m. Brazo de palanca de wz: Lx + Ly.
	 * Orden: fl (rueda 1), rl (2), rr (3), fr (4). */
	cfg->wheel_radius = 0.0762f;
	cfg->half_length = 0.1583f;
	cfg->half_width = 0.1734f;
	/* E6B2-CWZ de 500 PPR x4 montado en el eje de la rueda (instructions.md,
	 * nov 2026): una vuelta de rueda = 1 del encoder = 2 del motor. */
	cfg->counts_per_rev = 2000.0f;
	/* Motor de 165 rpm -> 82.5 rpm de rueda = 8.64 rad/s (2750 QPPS), por
	 * debajo del límite de 5000 QPPS de las RoboClaw. Margen del 10 %. */
	cfg->max_wheel_rad_s = 7.7f;
	cfg->lin_acc = 1.0f;
	cfg->ang_acc = 3.0f;
	cfg->rc_accel_qpps = 20000;
	cfg->rc_addr[0] = 128;
	cfg->rc_addr[1] = 129;
	/* RoboClaw 128 (frente): M1 = rueda 1 (fl), M2 = rueda 4 (fr).
	 * RoboClaw 129 (atrás):  M1 = rueda 2 (rl), M2 = rueda 3 (rr).
	 * Signos a confirmar con la prueba de ruedas en el aire (docs/BRINGUP.md). */
	cfg->map[0] = (amr_wheel_map_t){0, 0, 1};
	cfg->map[1] = (amr_wheel_map_t){1, 0, 1};
	cfg->map[2] = (amr_wheel_map_t){1, 1, 1};
	cfg->map[3] = (amr_wheel_map_t){0, 1, 1};
	cfg->fault_after = 3;
	/* Las RoboClaw vienen con 3 s; a 50 Hz de comandos, 0.5 s basta y detiene
	 * las ruedas pronto si el MCU se cuelga (el FS26 lo reinicia aparte). */
	cfg->rc_serial_timeout_ds = 5;
}

bool amr_drive_init(amr_drive_t *d, const amr_drive_cfg_t *cfg, roboclaw_t *rc)
{
	memset(d, 0, sizeof(*d));
	d->cfg = *cfg;
	d->rc = rc;
	if (!(cfg->counts_per_rev > 0.0f) || !(cfg->max_wheel_rad_s > 0.0f) || rc == NULL) {
		return false;
	}
	for (int i = 0; i < AMR_OMNI_WHEELS; i++) {
		if (cfg->map[i].rc > 1U || cfg->map[i].motor > 1U ||
		    (cfg->map[i].sign != 1 && cfg->map[i].sign != -1)) {
			return false;
		}
	}
	return amr_mecanum_init(&d->kin, cfg->wheel_radius, cfg->half_length, cfg->half_width);
}

static float ramp(float value, float target, float step)
{
	if (value < target) {
		return (target - value) < step ? target : value + step;
	}
	return (value - target) < step ? target : value - step;
}

/* Escala (vx, vy, wz) para que ninguna rueda supere max_wheel_rad_s. */
static void saturate(const amr_drive_t *d, float *vx, float *vy, float *wz)
{
	float w[AMR_OMNI_WHEELS], peak = 0.0f;

	amr_omni_inverse(&d->kin, *vx, *vy, *wz, w);
	for (int i = 0; i < AMR_OMNI_WHEELS; i++) {
		peak = fabsf(w[i]) > peak ? fabsf(w[i]) : peak;
	}
	if (peak > d->cfg.max_wheel_rad_s) {
		float s = d->cfg.max_wheel_rad_s / peak;
		*vx *= s;
		*vy *= s;
		*wz *= s;
	}
}

int32_t amr_drive_qpps(const amr_drive_t *d, int wheel, float rad_s)
{
	float q = rad_s * d->cfg.counts_per_rev / TWO_PI_F * (float)d->cfg.map[wheel].sign;

	if (q > 2.0e9f) {
		q = 2.0e9f;
	} else if (q < -2.0e9f) {
		q = -2.0e9f;
	}
	return (int32_t)lroundf(q);
}

static void mark(amr_drive_t *d, int rc_index, bool ok)
{
	uint32_t bit = rc_index == 0 ? AMR_DRIVE_FAULT_RC0 : AMR_DRIVE_FAULT_RC1;

	if (!ok) {
		d->timeout_set[rc_index] = false;
	}
	if (ok) {
		d->fail_count[rc_index] = 0;
		d->faults &= ~bit;
	} else if (d->fail_count[rc_index] < 255U && ++d->fail_count[rc_index] >= d->cfg.fault_after) {
		d->faults |= bit;
	}
}

void amr_drive_step(amr_drive_t *d, float vx, float vy, float wz, float dt,
		    amr_drive_meas_t *meas)
{
	float w[AMR_OMNI_WHEELS];
	int32_t speed[2][2] = {{0, 0}, {0, 0}};
	int32_t enc[2][2];
	bool enc_ok[2];

	memset(meas, 0, sizeof(*meas));
	if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) {
		vx = vy = wz = 0.0f;
	}
	/* 1) Saturación proporcional de la CONSIGNA: conserva la dirección. Como el
	 *    conjunto de velocidades alcanzables es convexo, todo punto de la rampa
	 *    entre el estado actual y la consigna también es alcanzable. */
	saturate(d, &vx, &vy, &wz);
	/* 2) Rampa vectorial en (vx, vy) para no deformar la dirección; escalar en wz. */
	if (dt > 0.0f) {
		float dvx = vx - d->vx, dvy = vy - d->vy;
		float mag = sqrtf(dvx * dvx + dvy * dvy), step = d->cfg.lin_acc * dt;
		if (mag > step) {
			dvx *= step / mag;
			dvy *= step / mag;
		}
		d->vx += dvx;
		d->vy += dvy;
		d->wz = ramp(d->wz, wz, d->cfg.ang_acc * dt);
	}
	amr_omni_inverse(&d->kin, d->vx, d->vy, d->wz, w);
	for (int i = 0; i < AMR_OMNI_WHEELS; i++) {
		const amr_wheel_map_t *m = &d->cfg.map[i];
		speed[m->rc][m->motor] = amr_drive_qpps(d, i, w[i]);
	}
	meas->cmd_vx = d->vx;
	meas->cmd_vy = d->vy;
	meas->cmd_wz = d->wz;

	for (int r = 0; r < 2; r++) {
		/* Se programa (y re-programa tras un hueco: la RoboClaw pudo
		 * reiniciarse y volver a su valor de EEPROM) antes de mover. */
		if (d->cfg.rc_serial_timeout_ds > 0U && !d->timeout_set[r]) {
			d->timeout_set[r] = rc_set_serial_timeout(d->rc, d->cfg.rc_addr[r],
								  d->cfg.rc_serial_timeout_ds) == RC_OK;
		}
		rc_status_t st = rc_speed_accel_m1m2(d->rc, d->cfg.rc_addr[r], d->cfg.rc_accel_qpps,
						     speed[r][0], speed[r][1]);
		rc_status_t se = rc_read_encoders(d->rc, d->cfg.rc_addr[r], &enc[r][0], &enc[r][1]);
		enc_ok[r] = (se == RC_OK);
		mark(d, r, st == RC_OK && se == RC_OK);
	}

	if (!(enc_ok[0] && enc_ok[1])) {
		d->enc_valid = false; /* tras un hueco, la próxima lectura solo re-inicializa */
		return;
	}
	for (int i = 0; i < AMR_OMNI_WHEELS; i++) {
		const amr_wheel_map_t *m = &d->cfg.map[i];
		int32_t now = enc[m->rc][m->motor];
		/* Diferencia modular: el desborde del contador de 32 bits no produce saltos. */
		int32_t delta = d->enc_valid ? (int32_t)((uint32_t)now - (uint32_t)d->enc_prev[i]) : 0;
		float drad = (float)delta * (float)m->sign * TWO_PI_F / d->cfg.counts_per_rev;

		d->enc_prev[i] = now;
		d->wheel_pos[i] += (double)drad;
		meas->wheel_pos[i] = (float)d->wheel_pos[i];
		meas->wheel_vel[i] = (d->enc_valid && dt > 0.0f) ? drad / dt : 0.0f;
	}
	meas->valid = d->enc_valid;
	d->enc_valid = true;
	if (meas->valid) {
		amr_omni_forward(&d->kin, meas->wheel_vel, &meas->vx, &meas->vy, &meas->wz);
		meas->residual = amr_omni_residual(&d->kin, meas->wheel_vel);
	}
}

void amr_drive_stop(amr_drive_t *d)
{
	d->vx = d->vy = d->wz = 0.0f;
	/* Sin lectura de encoders mientras está detenida: al reanudar, la primera
	 * lectura solo re-sincroniza (sin un pico de velocidad por el hueco). */
	d->enc_valid = false;
	for (int r = 0; r < 2; r++) {
		mark(d, r, rc_duty_m1m2(d->rc, d->cfg.rc_addr[r], 0, 0) == RC_OK);
	}
}
