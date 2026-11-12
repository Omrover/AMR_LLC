#include "llc.h"

#include <math.h>
#include <string.h>

#define MAX_DT_S 0.2f

static const amr_cmd_kind_t ack_kind[LLC_CMD_COUNT] = {
	[LLC_CMD_ARM] = AMR_CMD_ARM,
	[LLC_CMD_GRIPPER] = AMR_CMD_GRIPPER,
	[LLC_CMD_ESTOP] = AMR_CMD_ESTOP,
	[LLC_CMD_ODOM_RESET] = AMR_CMD_ODOM_RESET,
};

void llc_default_cfg(llc_cfg_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	amr_drive_default_cfg(&cfg->drive);
	amr_ekf_default_cfg(&cfg->ekf);
	grip_default_cfg(&cfg->grip);
	gim_arm_default_cfg(&cfg->arm);
	cfg->can_timeout_ms = 3;      /* respuesta de 8 bytes a 1 Mbps: ~0.2 ms */
	cfg->rc_timeout_ms = 10;      /* respuesta de 10 bytes a 115200: ~1 ms */
	cfg->rc_retries = 1;
	cfg->imu_timeout_ms = 200;
	cfg->imu_z_sign = 1.0f;
	cfg->battery_critical_v = 19.8f; /* 6S: 3.3 V/celda */
	cfg->slow_period_ms = 1000;
}

bool llc_init(llc_t *l, const llc_cfg_t *cfg, const rc_transport_t *rc_t,
	      const gim_transport_t *can_t, const llc_io_t *io)
{
	memset(l, 0, sizeof(*l));
	l->cfg = *cfg;
	l->io = *io;
	rc_init(&l->rc, rc_t, cfg->rc_timeout_ms, cfg->rc_retries);
	l->drive_ok = amr_drive_init(&l->drive, &cfg->drive, &l->rc);
	amr_ekf_init(&l->ekf, &cfg->ekf);
	grip_init(&l->grip, &cfg->grip);
	l->arm_ok = can_t != NULL && can_t->send != NULL && can_t->recv != NULL;
	if (l->arm_ok) {
		gim_init(&l->can, can_t, cfg->can_timeout_ms);
	}
	gim_arm_init(&l->arm, &l->can, &cfg->arm);
	amr_vel_wd_init(&l->wd);
	l->batt_v = -1.0f;
	l->logic_v = -1.0f;
	l->servo_us = grip_pulse_us(&l->grip);
	if (l->io.servo_set_us != NULL) {
		(void)l->io.servo_set_us(l->io.ctx, l->servo_us);
	}
	return l->drive_ok;
}

/* ------------------------------------------------------------------ comandos */

typedef union {
	amr_cmd_arm_t arm;
	amr_cmd_gripper_t grip;
	amr_cmd_estop_t estop;
	amr_cmd_odom_reset_t odom;
} any_cmd_t;

static bool parse(llc_cmd_topic_t topic, const char *json, size_t len, const char *boot_id,
		  any_cmd_t *c, uint32_t *seq)
{
	bool ok = false;

	switch (topic) {
	case LLC_CMD_ARM:
		ok = amr_parse_cmd_arm(json, len, boot_id, &c->arm);
		*seq = c->arm.seq;
		break;
	case LLC_CMD_GRIPPER:
		ok = amr_parse_cmd_gripper(json, len, boot_id, &c->grip);
		*seq = c->grip.seq;
		break;
	case LLC_CMD_ESTOP:
		ok = amr_parse_cmd_estop(json, len, boot_id, &c->estop);
		*seq = c->estop.seq;
		break;
	case LLC_CMD_ODOM_RESET:
		ok = amr_parse_cmd_odom_reset(json, len, boot_id, &c->odom);
		*seq = c->odom.seq;
		break;
	default:
		break;
	}
	return ok;
}

static amr_ack_result_t execute(llc_t *l, llc_cmd_topic_t topic, const any_cmd_t *c,
				const char **msg)
{
	switch (topic) {
	case LLC_CMD_ARM: {
		static const gim_arm_op_t ops[] = {
			[AMR_ARM_GOAL] = GIM_ARM_OP_GOAL,     [AMR_ARM_ENABLE] = GIM_ARM_OP_ENABLE,
			[AMR_ARM_DISABLE] = GIM_ARM_OP_DISABLE, [AMR_ARM_CLEAR] = GIM_ARM_OP_CLEAR,
			[AMR_ARM_HOME] = GIM_ARM_OP_HOME,
		};

		if (!l->arm_ok) {
			if (c->arm.op == AMR_ARM_DISABLE || c->arm.op == AMR_ARM_CLEAR) {
				return AMR_ACK_OK; /* no mueven nada */
			}
			*msg = "sin bus CAN del brazo";
			return AMR_ACK_REJECTED;
		}
		if ((unsigned int)c->arm.op >= sizeof(ops) / sizeof(ops[0])) {
			*msg = "op desconocida";
			return AMR_ACK_REJECTED;
		}
		return gim_arm_command(&l->arm, ops[c->arm.op], c->arm.pos, c->arm.acc, c->arm.dec,
				       msg)
			       ? AMR_ACK_OK
			       : AMR_ACK_REJECTED;
	}
	case LLC_CMD_GRIPPER:
		if (l->estop) {
			*msg = "e-stop activo";
			return AMR_ACK_REJECTED;
		}
		grip_command(&l->grip, c->grip.pos, c->grip.max_cur);
		return AMR_ACK_OK;
	case LLC_CMD_ESTOP:
		l->estop = c->estop.active;
		if (!l->estop) {
			l->estop_applied = false;
		}
		return AMR_ACK_OK;
	case LLC_CMD_ODOM_RESET:
		amr_ekf_reset(&l->ekf, c->odom.x, c->odom.y, c->odom.yaw);
		return AMR_ACK_OK;
	default:
		*msg = "topic desconocido";
		return AMR_ACK_REJECTED;
	}
}

int llc_command(llc_t *l, llc_cmd_topic_t topic, const char *json, size_t len, bool retain,
		const amr_hdr_t *hdr, uint32_t now_ms, char *ack, size_t ack_size)
{
	any_cmd_t c;
	uint32_t seq = 0;
	const char *msg = NULL;
	amr_ack_result_t result;
	int n;

	if (topic == LLC_CMD_VEL) {
		amr_cmd_vel_t v;

		if (!retain && amr_parse_cmd_vel(json, len, hdr->boot_id, &v)) {
			amr_vel_wd_accept(&l->wd, &v, now_ms);
		} else {
			amr_vel_wd_reject(&l->wd); /* inválido: nunca deja la base en movimiento */
		}
		return 0;
	}
	if ((unsigned int)topic >= (unsigned int)LLC_CMD_COUNT) {
		return 0;
	}
	if (retain || !parse(topic, json, len, hdr->boot_id, &c, &seq)) {
		if (!amr_peek_seq(json, len, &seq)) {
			return 0; /* sin seq no hay a quién responder */
		}
		result = AMR_ACK_REJECTED;
		msg = retain ? "comando con retain" : "comando invalido";
	} else if (!amr_seq_accept(&l->seq[topic], seq)) {
		result = AMR_ACK_DUPLICATE;
	} else {
		result = execute(l, topic, &c, &msg);
	}
	n = amr_fmt_ack(ack, ack_size, hdr, seq, ack_kind[topic], result, msg);
	return n > 0 ? n : 0;
}

/* -------------------------------------------------------------------- control */

static void update_faults(llc_t *l, uint32_t now_ms)
{
	uint32_t f = l->faults & ~(uint32_t)(AMR_FAULT_ROBOCLAW_128 | AMR_FAULT_ROBOCLAW_129 |
					      AMR_FAULT_IMU | AMR_FAULT_GRIPPER_OVERCURRENT |
					      AMR_FAULT_BATTERY_CRITICAL);

	if (!l->drive_ok) {
		f |= AMR_FAULT_ROBOCLAW_128 | AMR_FAULT_ROBOCLAW_129;
	} else {
		/* bits 0/1 de amr_drive = bits 0/1 del contrato */
		f |= l->drive.faults & (AMR_FAULT_ROBOCLAW_128 | AMR_FAULT_ROBOCLAW_129);
	}
	if (!l->imu_valid || (uint32_t)(now_ms - l->imu_ms) > l->cfg.imu_timeout_ms) {
		f |= AMR_FAULT_IMU;
	}
	if (l->grip.overcurrent) {
		f |= AMR_FAULT_GRIPPER_OVERCURRENT;
	}
	if (l->batt_valid && l->batt_v < l->cfg.battery_critical_v) {
		f |= AMR_FAULT_BATTERY_CRITICAL;
	}
	f &= ~(uint32_t)(AMR_FAULT_CAN_BUS | AMR_FAULT_ARM_MOTOR);
	if (l->arm_ok && gim_arm_bus_fault(&l->arm)) {
		f |= AMR_FAULT_CAN_BUS;
	}
	if (l->arm_ok && gim_arm_motor_fault(&l->arm)) {
		f |= AMR_FAULT_ARM_MOTOR;
	}
	l->faults = f;
}

static void slow_step(llc_t *l, uint32_t now_ms)
{
	float v;

	if (l->stepped && (uint32_t)(now_ms - l->slow_ms) < l->cfg.slow_period_ms) {
		return;
	}
	l->slow_ms = now_ms;
	/* Ambas RoboClaw comparten la batería: se lee de la primera que responda. */
	for (int r = 0; r < 2; r++) {
		if (rc_read_main_battery(&l->rc, l->cfg.drive.rc_addr[r], &v) == RC_OK) {
			l->batt_v = v;
			l->batt_valid = true;
			if (rc_read_logic_battery(&l->rc, l->cfg.drive.rc_addr[r], &v) == RC_OK) {
				l->logic_v = v;
			}
			break;
		}
	}
}

static void fuse(llc_t *l, uint32_t now_ms, float dt)
{
	bool imu_fresh = l->imu_valid && (uint32_t)(now_ms - l->imu_ms) <= l->cfg.imu_timeout_ms;
	float s = l->cfg.imu_z_sign;

	amr_ekf_predict(&l->ekf, dt);
	if (imu_fresh && l->imu_new_gyro) {
		(void)amr_ekf_update_gyro(&l->ekf, s * l->imu.gyro[2]);
	}
	if (imu_fresh && l->imu_new_quat) {
		float yaw = amr_quat_yaw(l->imu.quat[0], l->imu.quat[1], l->imu.quat[2],
					 l->imu.quat[3]);
		(void)amr_ekf_update_imu_yaw(&l->ekf, s * yaw);
	}
	l->imu_new_gyro = false;
	l->imu_new_quat = false;
	if (l->meas.valid) {
		(void)amr_ekf_update_wheels(&l->ekf, l->meas.vx, l->meas.vy, l->meas.wz);
	}
}

static void step_gripper(llc_t *l, uint32_t dt_ms)
{
	float mv = 0.0f;
	bool ok = l->io.adc_read_mv != NULL && l->io.adc_read_mv(l->io.ctx, &mv) == 0;
	uint32_t us = grip_step(&l->grip, dt_ms, mv, ok);

	if (us != l->servo_us && l->io.servo_set_us != NULL &&
	    l->io.servo_set_us(l->io.ctx, us) == 0) {
		l->servo_us = us;
	}
}

void llc_control_step(llc_t *l, uint32_t now_ms, bool link_ok)
{
	uint32_t dt_ms = l->stepped ? (uint32_t)(now_ms - l->last_ms) : 0U;
	float dt = (float)dt_ms * 1e-3f;
	float vx, vy, wz;

	if (dt > MAX_DT_S) {
		dt = MAX_DT_S; /* el hilo se atrasó: no extrapolar de más */
	}
	amr_vel_wd_step(&l->wd, now_ms, link_ok, l->estop, &vx, &vy, &wz);

	if (l->drive_ok) {
		if (l->estop) {
			/* duty 0 en cada ciclo (rueda libre); además mantiene vivo el
			 * timeout serial de las RoboClaw */
			amr_drive_stop(&l->drive);
			memset(&l->meas, 0, sizeof(l->meas));
			l->estop_applied = true;
		} else {
			amr_drive_step(&l->drive, vx, vy, wz, dt, &l->meas);
			if (l->meas.valid) {
				l->started = true;
			}
		}
		for (int r = 0; r < 2; r++) {
			if (l->drive.fail_count[r] > l->prev_fail[r]) {
				l->rc_err[r]++;
			}
			l->prev_fail[r] = l->drive.fail_count[r];
		}
	}
	if (l->arm_ok) {
		/* E-stop: el brazo se queda donde está (con 0xCF caería por gravedad). */
		gim_arm_set_hold(&l->arm, l->estop);
		gim_arm_step(&l->arm, now_ms);
	}
	slow_step(l, now_ms);
	if (l->stepped) {
		fuse(l, now_ms, dt);
	}
	step_gripper(l, dt_ms);
	update_faults(l, now_ms);
	l->last_ms = now_ms;
	l->stepped = true;
}

/* ----------------------------------------------------------------- telemetría */

static amr_state_t state_of(const llc_t *l)
{
	if (l->estop) {
		return AMR_STATE_ESTOP;
	}
	if (l->faults & (AMR_FAULT_ROBOCLAW_128 | AMR_FAULT_ROBOCLAW_129)) {
		return AMR_STATE_FAULT;
	}
	return l->started ? AMR_STATE_OK : AMR_STATE_INIT;
}

int llc_telemetry(const llc_t *l, llc_tel_t kind, const amr_hdr_t *hdr, char *buf, size_t size)
{
	switch (kind) {
	case LLC_TEL_ODOM: {
		amr_odom_t m;

		m.x = l->ekf.s[AMR_EKF_X];
		m.y = l->ekf.s[AMR_EKF_Y];
		m.yaw = amr_wrap_pi(l->ekf.s[AMR_EKF_YAW]);
		m.vx = l->ekf.s[AMR_EKF_VX];
		m.vy = l->ekf.s[AMR_EKF_VY];
		m.wz = l->ekf.s[AMR_EKF_WZ];
		amr_ekf_variances(&l->ekf, m.cov);
		return amr_fmt_odom(buf, size, hdr, &m);
	}
	case LLC_TEL_IMU: {
		amr_imu_t m;

		if (!l->imu_valid) {
			return -1;
		}
		memcpy(m.q, l->imu.quat, sizeof(m.q));
		memcpy(m.gyro, l->imu.gyro, sizeof(m.gyro));
		memcpy(m.acc, l->imu.accel, sizeof(m.acc));
		m.acc_status = l->imu.accel_accuracy;
		return amr_fmt_imu(buf, size, hdr, &m);
	}
	case LLC_TEL_WHEELS: {
		amr_wheels_t m;

		for (int i = 0; i < AMR_N_WHEELS; i++) {
			m.pos[i] = (float)l->drive.wheel_pos[i];
			m.vel[i] = l->meas.valid ? l->meas.wheel_vel[i] : 0.0f;
		}
		m.rc_err[0] = l->rc_err[0];
		m.rc_err[1] = l->rc_err[1];
		return amr_fmt_wheels(buf, size, hdr, &m);
	}
	case LLC_TEL_ARM: {
		amr_arm_t m;

		memset(&m, 0, sizeof(m));
		m.enabled = l->arm.enabled;
		for (int i = 0; i < AMR_N_JOINTS && i < GIM_ARM_JOINTS; i++) {
			const gim_joint_t *j = &l->arm.j[i];

			m.pos[i] = j->pos;
			m.vel_rpm[i] = j->vel_rpm;
			m.cur_a[i] = j->cur_a;
			m.temp_c[i] = j->temp_c;
			m.in_pos[i] = gim_arm_in_pos(&l->arm, i);
			m.fault[i] = j->online ? j->fault : 0xFF; /* 0xFF: sin respuesta por CAN */
		}
		m.grip_pos = l->grip.pos;
		m.grip_cur_a = l->grip.cur_a;
		m.grasped = l->grip.grasped;
		return amr_fmt_arm(buf, size, hdr, &m);
	}
	case LLC_TEL_BATTERY: {
		amr_battery_t m = {l->batt_v, l->logic_v};

		if (!l->batt_valid) {
			return -1;
		}
		return amr_fmt_battery(buf, size, hdr, &m);
	}
	case LLC_TEL_STATUS: {
		amr_status_t m;

		m.state = state_of(l);
		m.estop = l->estop;
		m.faults = l->faults;
		m.wd_tripped = l->wd.tripped;
		m.cpu = l->cpu < 0.0f ? 0.0f : (l->cpu > 1.0f ? 1.0f : l->cpu);
		return amr_fmt_status(buf, size, hdr, &m);
	}
	default:
		return -1;
	}
}

/* ------------------------------------------------------------------------ IMU */

void llc_imu_init(llc_imu_t *imu, const bno_transport_t *t)
{
	memset(imu, 0, sizeof(*imu));
	bno_init(&imu->bno, t);
	imu->grv_interval_us = 10000;   /* 100 Hz, igual que llc/imu */
	imu->gyro_interval_us = 10000;
	imu->accel_interval_us = 20000;
}

static int imu_configure(llc_imu_t *imu)
{
	if (bno_enable_report(&imu->bno, BNO_RPT_GAME_ROTATION_VECTOR, imu->grv_interval_us) < 0 ||
	    bno_enable_report(&imu->bno, BNO_RPT_GYRO, imu->gyro_interval_us) < 0 ||
	    bno_enable_report(&imu->bno, BNO_RPT_ACCEL, imu->accel_interval_us) < 0) {
		return -1;
	}
	return 0;
}

int llc_imu_service(llc_imu_t *imu, uint32_t now_ms, int max_packets)
{
	uint32_t upd = 0;

	/* Sin reportes tras el arranque: se configuran (y se reintenta cada 1 s si
	 * el sensor no contesta o se perdió el anuncio de reset). */
	if (!imu->configured &&
	    (!imu->attempted || (uint32_t)(now_ms - imu->enable_ms) >= 1000U)) {
		imu->attempted = true;
		imu->enable_ms = now_ms;
		if (imu_configure(imu) == 0) {
			imu->configured = true;
		}
	}
	for (int i = 0; i < max_packets; i++) {
		int rc = bno_service(&imu->bno);

		if (rc < 0) {
			imu->configured = false;
			return upd ? (int)upd : rc;
		}
		if (rc == 0) {
			break;
		}
		if (rc & BNO_EVT_RESET) {
			/* el sensor se reinició: reportes deshabilitados, configurar ya */
			imu->configured = imu_configure(imu) == 0;
			imu->enable_ms = now_ms;
			continue;
		}
		upd |= (uint32_t)rc & (BNO_UPD_QUAT | BNO_UPD_GYRO | BNO_UPD_ACCEL | BNO_UPD_LIN_ACCEL);
	}
	return (int)upd;
}

void llc_imu_sample(llc_t *l, const bno08x_data_t *d, uint32_t upd, uint32_t now_ms)
{
	if ((upd & (BNO_UPD_QUAT | BNO_UPD_GYRO)) == 0U) {
		return;
	}
	l->imu = *d;
	l->imu_valid = true;
	l->imu_ms = now_ms;
	l->imu_new_quat |= (upd & BNO_UPD_QUAT) != 0U;
	l->imu_new_gyro |= (upd & BNO_UPD_GYRO) != 0U;
}
