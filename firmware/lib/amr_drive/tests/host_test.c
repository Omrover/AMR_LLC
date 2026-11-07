/* Pruebas en host de amr_drive con 2 RoboClaw simuladas en el mismo bus. */
#include "amr_drive.h"
#include "roboclaw_sim.h"

#include <math.h>
#include <stdio.h>

static int failures;
#define CHECK(cond)                                                               \
	do {                                                                      \
		if (!(cond)) {                                                    \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
			failures++;                                               \
		}                                                                 \
	} while (0)
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

typedef struct {
	rc_sim_t sim;
	roboclaw_t rc;
	amr_drive_t drive;
	amr_drive_cfg_t cfg;
} rig_t;

static void rig_init(rig_t *r)
{
	rc_transport_t t;

	rc_sim_init(&r->sim, 128, 129);
	t = rc_sim_transport(&r->sim);
	rc_init(&r->rc, &t, 10, 0);
	amr_drive_default_cfg(&r->cfg);
}

static void run(rig_t *r, float vx, float vy, float wz, double seconds, amr_drive_meas_t *m)
{
	const float dt = 0.02f;
	for (int i = 0; i < (int)lround(seconds / dt); i++) {
		rc_sim_step(&r->sim, dt);
		amr_drive_step(&r->drive, vx, vy, wz, dt, m);
	}
}

static void test_forward_motion(void)
{
	rig_t r;
	amr_drive_meas_t m;
	double w_fl;

	rig_init(&r);
	w_fl = -sin(M_PI / 4) * 0.3 / r.cfg.wheel_radius; /* rad/s */
	CHECK(amr_drive_init(&r.drive, &r.cfg, &r.rc));
	run(&r, 0.3f, 0.0f, 0.0f, 2.0, &m);
	CHECK(m.valid);
	CHECK(NEAR(m.cmd_vx, 0.3, 1e-6));
	CHECK(NEAR(m.vx, 0.3, 0.02) && NEAR(m.vy, 0.0, 0.02) && NEAR(m.wz, 0.0, 0.05));
	CHECK(m.residual < 0.01f);
	/* fl = RoboClaw 128 M1: QPPS = w * counts_per_rev / 2pi */
	CHECK(NEAR(r.sim.dev[0].target[0], w_fl * r.cfg.counts_per_rev / (2 * M_PI), 1.0));
	/* fr = RoboClaw 128 M2, rl = 129 M1, rr = 129 M2 (cableado de routing) */
	CHECK(NEAR(r.sim.dev[0].target[1], -w_fl * r.cfg.counts_per_rev / (2 * M_PI), 1.0));
	CHECK(NEAR(r.sim.dev[1].target[0], w_fl * r.cfg.counts_per_rev / (2 * M_PI), 1.0));
	CHECK(NEAR(r.sim.dev[1].target[1], -w_fl * r.cfg.counts_per_rev / (2 * M_PI), 1.0));
	CHECK(NEAR(m.wheel_vel[0], w_fl, 0.2));
	CHECK(r.drive.faults == 0);
}

static void test_ramp_and_saturation(void)
{
	rig_t r;
	amr_drive_meas_t m;
	float w[4];

	rig_init(&r);
	amr_drive_init(&r.drive, &r.cfg, &r.rc);
	run(&r, 1.0f, 0.0f, 0.0f, 0.1, &m); /* 5 ciclos con lin_acc = 1 m/s^2 */
	CHECK(NEAR(m.cmd_vx, 0.1, 1e-4));
	run(&r, 0.0f, 0.0f, 0.0f, 1.0, &m);
	run(&r, 0.3f, 0.4f, 0.0f, 0.1, &m); /* rampa vectorial: 0.1 m/s en dirección (0.6, 0.8) */
	CHECK(NEAR(m.cmd_vx, 0.06, 1e-4) && NEAR(m.cmd_vy, 0.08, 1e-4));

	run(&r, 5.0f, 2.0f, 0.0f, 10.0, &m); /* pide más de lo que las ruedas dan */
	amr_omni_inverse(&r.drive.kin, m.cmd_vx, m.cmd_vy, m.cmd_wz, w);
	for (int i = 0; i < 4; i++) {
		CHECK(fabsf(w[i]) <= r.cfg.max_wheel_rad_s + 1e-3f);
	}
	CHECK(NEAR(m.cmd_vy / m.cmd_vx, 2.0 / 5.0, 0.02)); /* conserva la dirección */
}

static void test_sign_mapping(void)
{
	rig_t r;
	amr_drive_meas_t m;

	rig_init(&r);
	r.cfg.map[0].sign = -1; /* motor fl montado invertido */
	amr_drive_init(&r.drive, &r.cfg, &r.rc);
	run(&r, 0.3f, 0.0f, 0.0f, 2.0, &m);
	CHECK(r.sim.dev[0].target[0] > 0.0);         /* el motor gira al revés... */
	CHECK(m.wheel_vel[0] < 0.0f);                /* ...pero la rueda reporta su convención */
	CHECK(NEAR(m.vx, 0.3, 0.02));
}

static void test_rotation(void)
{
	rig_t r;
	amr_drive_meas_t m;

	rig_init(&r);
	amr_drive_init(&r.drive, &r.cfg, &r.rc);
	run(&r, 0.0f, 0.0f, 1.0f, 2.0, &m);
	CHECK(NEAR(m.wz, 1.0, 0.05));
	for (int i = 0; i < 4; i++) {
		CHECK(NEAR(m.wheel_vel[i], r.cfg.center_distance / r.cfg.wheel_radius, 0.2));
	}
}

static void test_faults(void)
{
	rig_t r;
	amr_drive_meas_t m;

	rig_init(&r);
	amr_drive_init(&r.drive, &r.cfg, &r.rc);
	run(&r, 0.2f, 0.0f, 0.0f, 0.5, &m);
	r.sim.dev[1].present = false; /* RoboClaw 129 desconectada */
	run(&r, 0.2f, 0.0f, 0.0f, 0.1, &m);
	CHECK(!m.valid);
	CHECK(r.drive.faults == AMR_DRIVE_FAULT_RC1);
	r.sim.dev[1].present = true;
	run(&r, 0.2f, 0.0f, 0.0f, 0.04, &m); /* 1er ciclo re-inicializa, 2do mide */
	CHECK(m.valid && r.drive.faults == 0);
	CHECK(NEAR(m.vx, 0.2, 0.02));
}

static void test_encoder_wrap(void)
{
	rig_t r;
	amr_drive_meas_t m;
	float before;

	rig_init(&r);
	amr_drive_init(&r.drive, &r.cfg, &r.rc);
	run(&r, 0.3f, 0.0f, 0.0f, 1.0, &m);
	before = m.wheel_pos[2];
	/* contadores a punto de desbordar */
	r.sim.dev[1].enc[1] += 4294967296.0 * 0.5 - r.sim.dev[1].enc[1] - 50.0; /* rr = 129 M2 */
	r.drive.enc_valid = false; /* simula re-sincronización */
	run(&r, 0.3f, 0.0f, 0.0f, 1.0, &m);
	CHECK(m.valid);
	CHECK(NEAR(m.wheel_pos[2] - before, m.wheel_vel[2] * 1.0, 0.3)); /* sin salto */
}

static void test_stop(void)
{
	rig_t r;
	amr_drive_meas_t m;

	rig_init(&r);
	amr_drive_init(&r.drive, &r.cfg, &r.rc);
	run(&r, 0.3f, 0.0f, 0.0f, 1.0, &m);
	amr_drive_stop(&r.drive);
	CHECK(r.sim.dev[0].duty_mode && r.sim.dev[1].duty_mode);
	CHECK(r.drive.vx == 0.0f);
}

static void test_invalid_cfg(void)
{
	rig_t r;

	rig_init(&r);
	r.cfg.map[2].motor = 2;
	CHECK(!amr_drive_init(&r.drive, &r.cfg, &r.rc));
	rig_init(&r);
	r.cfg.counts_per_rev = 0.0f;
	CHECK(!amr_drive_init(&r.drive, &r.cfg, &r.rc));
}

int main(void)
{
	test_forward_motion();
	test_ramp_and_saturation();
	test_sign_mapping();
	test_rotation();
	test_faults();
	test_encoder_wrap();
	test_stop();
	test_invalid_cfg();
	if (failures == 0) {
		printf("amr_drive: PASSED (0 failures)\n");
		return 0;
	}
	printf("amr_drive: FAILED (%d failures)\n", failures);
	return 1;
}
