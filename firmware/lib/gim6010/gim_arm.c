#include "gim_arm.h"

#include <math.h>
#include <string.h>

#define PEND_DISABLE (1u << 0)
#define PEND_CLEAR (1u << 1)
#define PEND_ACCDEC (1u << 2)
#define PEND_POS (1u << 3)

void gim_arm_default_cfg(gim_arm_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	for (int i = 0; i < GIM_ARM_JOINTS; i++) {
		cfg->addr[i] = (uint8_t)(i + 1);
		/* Una vuelta a cada lado: los límites finos (rad) los aplica la NUC. */
		cfg->pos_min[i] = -GIM_COUNTS_PER_REV;
		cfg->pos_max[i] = GIM_COUNTS_PER_REV;
	}
	cfg->in_pos_tol = 46; /* ~1 grado */
	cfg->default_acc_rpm_s = 100.0f;
	cfg->ae_period_ms = 500;
	cfg->offline_after = 3;
}

void gim_arm_init(gim_arm_t *a, gim_bus_t *bus, const gim_arm_cfg_t *cfg)
{
	memset(a, 0, sizeof(*a));
	a->cfg = *cfg;
	a->bus = bus;
	for (int i = 0; i < GIM_ARM_JOINTS; i++) {
		a->j[i].online = true; /* hasta que falle offline_after veces */
	}
}

static uint32_t crpm(float rpm_s)
{
	if (!(rpm_s > 0.0f)) {
		return 0;
	}
	return (uint32_t)lroundf(rpm_s * 100.0f);
}

static bool all_pos_valid(const gim_arm_t *a)
{
	for (int i = 0; i < GIM_ARM_JOINTS; i++) {
		if (!a->j[i].pos_valid) {
			return false;
		}
	}
	return true;
}

static void set_goal(gim_arm_t *a, int i, int32_t target, uint32_t acc, uint32_t dec)
{
	gim_joint_t *j = &a->j[i];

	j->target = target;
	j->has_target = true;
	j->acc_crpm_s = acc;
	j->dec_crpm_s = dec;
	j->pending |= PEND_ACCDEC | PEND_POS;
	j->pending &= (uint8_t)~PEND_DISABLE;
}

bool gim_arm_command(gim_arm_t *a, gim_arm_op_t op, const int32_t pos[GIM_ARM_JOINTS],
		     const float acc[GIM_ARM_JOINTS], const float dec[GIM_ARM_JOINTS],
		     const char **msg)
{
	uint32_t def = crpm(a->cfg.default_acc_rpm_s);

	switch (op) {
	case GIM_ARM_OP_DISABLE:
		a->enabled = false;
		for (int i = 0; i < GIM_ARM_JOINTS; i++) {
			a->j[i].pending = PEND_DISABLE;
			a->j[i].has_target = false;
		}
		return true;
	case GIM_ARM_OP_CLEAR:
		for (int i = 0; i < GIM_ARM_JOINTS; i++) {
			a->j[i].pending |= PEND_CLEAR;
		}
		return true;
	case GIM_ARM_OP_ENABLE:
		if (!all_pos_valid(a)) {
			*msg = "sin posicion de todas las juntas";
			return false;
		}
		/* Energiza manteniendo la posición actual: nada se mueve. */
		a->enabled = true;
		for (int i = 0; i < GIM_ARM_JOINTS; i++) {
			a->j[i].pending |= PEND_CLEAR;
			set_goal(a, i, a->j[i].pos, def, def);
		}
		return true;
	case GIM_ARM_OP_HOME:
	case GIM_ARM_OP_GOAL:
		if (a->hold) {
			*msg = "e-stop activo";
			return false;
		}
		if (!a->enabled) {
			*msg = "brazo deshabilitado (op enable primero)";
			return false;
		}
		for (int i = 0; i < GIM_ARM_JOINTS; i++) {
			int32_t t = op == GIM_ARM_OP_HOME ? 0 : pos[i];

			if (t < a->cfg.pos_min[i] || t > a->cfg.pos_max[i]) {
				*msg = "objetivo fuera de los limites del MCU";
				return false;
			}
			if (op == GIM_ARM_OP_GOAL && (crpm(acc[i]) == 0U || crpm(dec[i]) == 0U)) {
				*msg = "acc/dec invalidos";
				return false;
			}
		}
		for (int i = 0; i < GIM_ARM_JOINTS; i++) {
			if (op == GIM_ARM_OP_HOME) {
				set_goal(a, i, 0, def, def);
			} else {
				set_goal(a, i, pos[i], crpm(acc[i]), crpm(dec[i]));
			}
		}
		return true;
	default:
		*msg = "op desconocida";
		return false;
	}
}

void gim_arm_set_hold(gim_arm_t *a, bool hold)
{
	if (hold && !a->hold) {
		for (int i = 0; i < GIM_ARM_JOINTS; i++) {
			gim_joint_t *j = &a->j[i];

			if (a->enabled && j->pos_valid) {
				/* Objetivo = posición medida: el perfil frena y se queda ahí. */
				j->target = j->pos;
				j->has_target = true;
				j->pending |= PEND_POS;
			}
		}
	}
	a->hold = hold;
}

static void mark(gim_arm_t *a, gim_joint_t *j, bool ok)
{
	if (ok) {
		j->fails = 0;
		j->online = true;
	} else if (j->fails < 255U && ++j->fails >= a->cfg.offline_after) {
		j->online = false;
	}
}

static void service(gim_arm_t *a, int i, bool read_ae)
{
	gim_joint_t *j = &a->j[i];
	uint8_t addr = a->cfg.addr[i];
	gim_a4_t a4;
	gim_status_t st;

	/* Un comando fallido queda pendiente y se reintenta el siguiente ciclo. */
	if (j->pending & PEND_DISABLE) {
		if (gim_disable(a->bus, addr) == GIM_OK) {
			j->pending &= (uint8_t)~PEND_DISABLE;
		}
	}
	if (j->pending & PEND_CLEAR) {
		if (gim_clear_fault(a->bus, addr) == GIM_OK) {
			j->pending &= (uint8_t)~PEND_CLEAR;
			j->fault = 0;
		}
	}
	if (j->pending & PEND_ACCDEC) {
		if (gim_trap_acc_dec(a->bus, addr, j->acc_crpm_s, j->dec_crpm_s) == GIM_OK) {
			j->pending &= (uint8_t)~PEND_ACCDEC;
		}
	}
	if ((j->pending & PEND_POS) && !(j->pending & PEND_ACCDEC)) {
		if (gim_pos_abs(a->bus, addr, j->target) == GIM_OK) {
			j->pending &= (uint8_t)~PEND_POS;
		}
	}

	st = gim_read_a4(a->bus, addr, &a4);
	mark(a, j, st == GIM_OK);
	if (st == GIM_OK) {
		j->pos = gim_unwrap(j->pos, j->pos_valid, a4.pos16);
		j->pos_valid = true;
		j->vel_rpm = (float)a4.vel_crpm / 100.0f;
		j->cur_a = (float)a4.cur_ma / 1000.0f;
		j->temp_c = (float)a4.temp_c;
	}
	if (read_ae) {
		gim_ae_t ae;

		if (gim_read_ae(a->bus, addr, &ae) == GIM_OK) {
			j->fault = ae.fault;
		}
	}
}

void gim_arm_step(gim_arm_t *a, uint32_t now_ms)
{
	bool read_ae = !a->stepped || (uint32_t)(now_ms - a->ae_ms) >= a->cfg.ae_period_ms;

	if (read_ae) {
		a->ae_ms = now_ms;
	}
	a->stepped = true;
	for (int i = 0; i < GIM_ARM_JOINTS; i++) {
		service(a, i, read_ae);
	}
}

bool gim_arm_in_pos(const gim_arm_t *a, int i)
{
	const gim_joint_t *j = &a->j[i];
	int32_t err;

	if (!j->pos_valid) {
		return false;
	}
	if (!j->has_target) {
		return true;
	}
	err = j->target - j->pos;
	return j->pending == 0U && err <= a->cfg.in_pos_tol && err >= -a->cfg.in_pos_tol &&
	       fabsf(j->vel_rpm) < 1.0f;
}

bool gim_arm_bus_fault(const gim_arm_t *a)
{
	for (int i = 0; i < GIM_ARM_JOINTS; i++) {
		if (!a->j[i].online) {
			return true;
		}
	}
	return false;
}

bool gim_arm_motor_fault(const gim_arm_t *a)
{
	for (int i = 0; i < GIM_ARM_JOINTS; i++) {
		if (a->j[i].fault != 0U) {
			return true;
		}
	}
	return false;
}
