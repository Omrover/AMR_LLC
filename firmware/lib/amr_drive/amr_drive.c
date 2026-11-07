#include "amr_drive.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F 6.28318530718f

void amr_drive_default_cfg(amr_drive_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->wheel_radius = 0.05f;         /* TBD */
	cfg->center_distance = 0.25f;      /* TBD */
	cfg->angles_rad[0] = 0.785398163f; /* fl 45 */
	cfg->angles_rad[1] = 2.35619449f;  /* rl 135 */
	cfg->angles_rad[2] = 3.92699082f;  /* rr 225 */
	cfg->angles_rad[3] = 5.49778714f;  /* fr 315 */
	cfg->counts_per_rev = 2400.0f;     /* TBD: E6B2-CWZ 600 PPR x4, sin reducción */
	cfg->max_wheel_rad_s = 30.0f;
	cfg->lin_acc = 1.0f;
	cfg->ang_acc = 3.0f;
	cfg->rc_accel_qpps = 20000;
	cfg->rc_addr[0] = 128;
	cfg->rc_addr[1] = 129;
	/* RoboClaw 128: M1 = fl, M2 = rl; RoboClaw 129: M1 = rr, M2 = fr (TBD cableado). */
	cfg->map[0] = (amr_wheel_map_t){0, 0, 1};
	cfg->map[1] = (amr_wheel_map_t){0, 1, 1};
	cfg->map[2] = (amr_wheel_map_t){1, 0, 1};
	cfg->map[3] = (amr_wheel_map_t){1, 1, 1};
	cfg->fault_after = 3;
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
	return amr_omni_init(&d->kin, cfg->wheel_radius, cfg->center_distance, cfg->angles_rad);
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
		d->wheel_pos[i] += drad;
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
	for (int r = 0; r < 2; r++) {
		mark(d, r, rc_duty_m1m2(d->rc, d->cfg.rc_addr[r], 0, 0) == RC_OK);
	}
}
