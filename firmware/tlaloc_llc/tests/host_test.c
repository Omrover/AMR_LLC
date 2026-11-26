/*
 * Prueba de lazo cerrado del núcleo llc en host: 2 RoboClaw, la BNO080 y 4
 * GIM6010-8 simulados a nivel de bytes/tramas, comandos JSON del contrato y
 * telemetría.
 */
#include "llc.h"
#include "bno08x_sim.h"
#include "gim6010_sim.h"
#include "roboclaw_sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(cond)                                                               \
	do {                                                                      \
		if (!(cond)) {                                                    \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
			failures++;                                               \
		}                                                                 \
	} while (0)
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))

#define BOOT "b00710c0ffee0001"

/* ------------------------------------------------------------ banco de prueba */

typedef struct {
	rc_sim_t rc;
	bno_sim_t bno;
	gim_sim_t gim;
	bool no_can;          /* rig sin bus CAN (brazo ausente) */
	bool estop_hw;        /* botón de paro físico abierto */
	llc_imu_t imu;
	llc_t llc;
	uint32_t now;
	/* io falso: motor DC del gripper (DRV8833) + ACS712 */
	float grip_duty;      /* última salida al puente H */
	bool grip_sleep;
	uint32_t grip_writes;
	float grip_true;      /* posición real 0..1 */
	float object_at;      /* posición del gripper donde toca la pieza (<0: sin pieza) */
	float stall_a;        /* corriente trabado con duty 1 */
	bool adc_ok;
	bool drv_fault;       /* nFAULT del DRV8833 */
	bool sensor_off;      /* ACS712 sin alimentar: el ADC lee 0 V */
	/* verdad del terreno */
	double x, y, yaw;
	uint32_t next_vel_ms;
	uint32_t seq;
	char ack[256];
	int ack_len;
} rig_t;

static const float zero_mv = 1650.0f;   /* 2.5 V del ACS712 a través del divisor 2/3 */

static int io_grip_drive(void *ctx, float duty, bool sleep)
{
	rig_t *r = ctx;

	r->grip_duty = duty;
	r->grip_sleep = sleep;
	r->grip_writes++;
	return 0;
}

static bool io_grip_fault(void *ctx)
{
	return ((rig_t *)ctx)->drv_fault;
}

static float grip_d(const rig_t *r)
{
	return r->grip_sleep ? 0.0f : r->grip_duty;
}

static bool grip_blocked(const rig_t *r, float d)
{
	return (d > 0.0f && r->grip_true >= 1.0f) || (d < 0.0f && r->grip_true <= 0.0f) ||
	       (d > 0.0f && r->object_at >= 0.0f && r->grip_true >= r->object_at);
}

/* Motor real: carrera completa en travel_ms a move_duty; trabado, I = |d| * stall_a. */
static void grip_motor_step(rig_t *r, double dt)
{
	float d = grip_d(r);
	const grip_cfg_t *c = &r->llc.cfg.grip;

	if (!grip_blocked(r, d)) {
		float v = d / c->move_duty / ((float)c->travel_ms * 1e-3f);

		r->grip_true += v * (float)dt;
		r->grip_true = r->grip_true < 0.0f ? 0.0f : (r->grip_true > 1.0f ? 1.0f : r->grip_true);
	}
}

static int io_adc(void *ctx, float *mv)
{
	rig_t *r = ctx;
	float d = grip_d(r);
	float amps;

	if (r->sensor_off) {
		*mv = 0.0f;
		return 0;
	}
	if (!r->adc_ok) {
		return -5;
	}
	amps = grip_blocked(r, d) ? fabsf(d) * r->stall_a
				  : (d != 0.0f ? 0.1f + 0.2f * fabsf(d) : 0.0f);
	*mv = zero_mv + amps * r->llc.cfg.grip.mv_per_a;
	return 0;
}

static amr_hdr_t hdr(rig_t *r)
{
	amr_hdr_t h = {BOOT, (int64_t)r->now};

	return h;
}

static void rig_init_ex(rig_t *r, bool can)
{
	llc_cfg_t cfg;
	rc_transport_t rt;
	bno_transport_t bt;
	gim_transport_t gt;
	llc_io_t io;

	memset(r, 0, sizeof(*r));
	r->no_can = !can;
	r->now = 1000;
	r->object_at = -1.0f;
	r->stall_a = 2.5f;
	r->adc_ok = true;
	r->grip_sleep = true;
	rc_sim_init(&r->rc, 128, 129);
	bno_sim_init(&r->bno);
	rt = rc_sim_transport(&r->rc);
	bt = bno_sim_transport(&r->bno);
	gim_sim_init(&r->gim);
	gt = gim_sim_transport(&r->gim);
	io.grip_drive = io_grip_drive;
	io.adc_read_mv = io_adc;
	io.grip_fault = io_grip_fault;
	io.ctx = r;
	llc_default_cfg(&cfg);
	CHECK(llc_init(&r->llc, &cfg, &rt, r->no_can ? NULL : &gt, &io));
	llc_imu_init(&r->imu, &bt);
}

static void rig_init(rig_t *r)
{
	rig_init_ex(r, true);
}

/* Velocidad real del cuerpo a partir de las ruedas simuladas. */
static void truth_twist(rig_t *r, float *vx, float *vy, float *wz)
{
	float w[4];
	const amr_drive_cfg_t *c = &r->llc.cfg.drive;

	for (int i = 0; i < 4; i++) {
		const amr_wheel_map_t *m = &c->map[i];

		w[i] = (float)(r->rc.dev[m->rc].speed[m->motor] * m->sign * 2.0 * M_PI /
			       c->counts_per_rev);
	}
	amr_omni_forward(&r->llc.drive.kin, w, vx, vy, wz);
}

static int send(rig_t *r, llc_cmd_topic_t topic, const char *json, bool retain)
{
	amr_hdr_t h = hdr(r);

	r->ack_len = llc_command(&r->llc, topic, json, strlen(json), retain, &h, r->now, r->ack,
				 sizeof(r->ack));
	return r->ack_len;
}

static void send_vel(rig_t *r, float vx, float vy, float wz)
{
	char json[200];

	snprintf(json, sizeof(json),
		 "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":%u,\"vx\":%.4f,\"vy\":%.4f,"
		 "\"wz\":%.4f,\"valid_ms\":250}",
		 ++r->seq, (double)vx, (double)vy, (double)wz);
	send(r, LLC_CMD_VEL, json, false);
}

/* Corre el sistema: física y IMU a 200 Hz, control a 50 Hz, cmd/vel a 20 Hz si
 * stream es true. */
static void run(rig_t *r, double seconds, bool stream, float vx, float vy, float wz)
{
	int ticks = (int)lround(seconds / 0.005);

	for (int k = 0; k < ticks; k++) {
		float tvx, tvy, twz;
		int upd;

		r->now += 5;
		rc_sim_step(&r->rc, 0.005);
		truth_twist(r, &tvx, &tvy, &twz);
		r->x += (tvx * cos(r->yaw) - tvy * sin(r->yaw)) * 0.005;
		r->y += (tvx * sin(r->yaw) + tvy * cos(r->yaw)) * 0.005;
		r->yaw += twz * 0.005;
		bno_sim_set_motion(&r->bno, (float)r->yaw, twz, 0.0f, 0.0f);
		bno_sim_step(&r->bno, 0.005);
		gim_sim_step(&r->gim, 0.005);
		grip_motor_step(r, 0.005);

		upd = llc_imu_service(&r->imu, r->now, 4);
		if (upd > 0) {
			llc_imu_sample(&r->llc, &r->imu.bno.data, (uint32_t)upd, r->now);
		}
		if (stream && (int32_t)(r->now - r->next_vel_ms) >= 0) {
			send_vel(r, vx, vy, wz);
			r->next_vel_ms = r->now + 50;
		}
		if (r->now % 20 == 0) {
			llc_estop_input(&r->llc, r->estop_hw);
			llc_control_step(&r->llc, r->now, true);
		}
	}
}

/* Extrae el número que sigue a "key": en un JSON plano. */
static double jnum(const char *json, const char *key)
{
	char pat[40];
	const char *p;

	snprintf(pat, sizeof(pat), "\"%s\":", key);
	p = strstr(json, pat);
	return p ? strtod(p + strlen(pat), NULL) : NAN;
}

static int tel(rig_t *r, llc_tel_t kind, char *buf, size_t size)
{
	amr_hdr_t h = hdr(r);

	return llc_telemetry(&r->llc, kind, &h, buf, size);
}

/* -------------------------------------------------------------------- pruebas */

static void test_boot_and_drive(void)
{
	rig_t r;
	char buf[600];

	rig_init(&r);
	CHECK(tel(&r, LLC_TEL_STATUS, buf, sizeof(buf)) > 0 && strstr(buf, "\"state\":\"init\""));
	run(&r, 0.5, false, 0, 0, 0);
	CHECK(r.imu.configured && r.bno.features_set >= 3);
	CHECK(tel(&r, LLC_TEL_STATUS, buf, sizeof(buf)) > 0);
	CHECK(strstr(buf, "\"state\":\"ok\"") && NEAR(jnum(buf, "faults"), 0, 0));
	CHECK(r.grip_sleep && r.llc.grip.state == GRIP_IDLE); /* gripper quieto, driver dormido */

	/* curva: avanza, se desliza y gira a la vez durante 4 s */
	run(&r, 4.0, true, 0.2f, 0.05f, 0.4f);
	CHECK(tel(&r, LLC_TEL_ODOM, buf, sizeof(buf)) > 0);
	CHECK(NEAR(jnum(buf, "x"), r.x, 0.02) && NEAR(jnum(buf, "y"), r.y, 0.02));
	CHECK(NEAR(jnum(buf, "yaw"), atan2(sin(r.yaw), cos(r.yaw)), 0.01));
	CHECK(NEAR(jnum(buf, "vx"), 0.2, 0.02) && NEAR(jnum(buf, "wz"), 0.4, 0.03));
	CHECK(tel(&r, LLC_TEL_WHEELS, buf, sizeof(buf)) > 0 && strstr(buf, "\"rc_err\":[0,0]"));
	CHECK(tel(&r, LLC_TEL_IMU, buf, sizeof(buf)) > 0);
	CHECK(tel(&r, LLC_TEL_BATTERY, buf, sizeof(buf)) > 0 && jnum(buf, "volt") > 20.0);
	printf("odom EKF vs. verdad tras 4 s de curva: x %.4f/%.4f  y %.4f/%.4f  yaw %.4f/%.4f\n",
	       (double)r.llc.ekf.s[AMR_EKF_X], r.x, (double)r.llc.ekf.s[AMR_EKF_Y], r.y,
	       (double)r.llc.ekf.s[AMR_EKF_YAW], r.yaw);
}

static void test_watchdog_and_invalid(void)
{
	rig_t r;
	char buf[400];

	rig_init(&r);
	run(&r, 0.3, false, 0, 0, 0);
	run(&r, 1.0, true, 0.3f, 0.0f, 0.0f);
	CHECK(NEAR(r.llc.meas.vx, 0.3, 0.03));
	run(&r, 0.25 + 0.3 + 0.1, false, 0, 0, 0); /* deja de enviar: expira y frena con rampa */
	CHECK(NEAR(r.llc.meas.vx, 0.0, 0.02));
	CHECK(tel(&r, LLC_TEL_STATUS, buf, sizeof(buf)) > 0 && strstr(buf, "\"wd\":true"));

	/* un cmd/vel con boot_id ajeno detiene de inmediato (sin esperar valid_ms) */
	run(&r, 1.0, true, 0.3f, 0.0f, 0.0f);
	send(&r, LLC_CMD_VEL,
	     "{\"v\":1,\"boot_id\":\"otro\",\"seq\":9,\"vx\":0.3,\"vy\":0,\"wz\":0,\"valid_ms\":250}",
	     false);
	run(&r, 0.02, false, 0, 0, 0);
	CHECK(r.llc.drive.vx < 0.3f);
	CHECK(!r.llc.wd.active);
}

static void test_commands_and_acks(void)
{
	rig_t r;
	char buf[400];
	const char *grip = "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":5,\"pos\":0.5,\"max_cur\":1.0}";

	rig_init(&r);
	run(&r, 0.6, false, 0, 0, 0);
	CHECK(send(&r, LLC_CMD_GRIPPER, grip, false) > 0 && strstr(r.ack, "\"result\":\"ok\""));
	CHECK(strstr(r.ack, "\"cmd\":\"gripper\"") && strstr(r.ack, "\"seq\":5"));
	CHECK(send(&r, LLC_CMD_GRIPPER, grip, false) > 0 && strstr(r.ack, "\"duplicate\""));
	CHECK(send(&r, LLC_CMD_GRIPPER, grip, true) > 0 && strstr(r.ack, "\"rejected\""));
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"ajeno\",\"seq\":6,\"active\":true}", false) > 0 &&
	      strstr(r.ack, "\"rejected\""));
	CHECK(!r.llc.estop);
	CHECK(send(&r, LLC_CMD_ARM,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":7,\"op\":\"enable\"}", false) > 0 &&
	      strstr(r.ack, "\"ok\""));
	CHECK(send(&r, LLC_CMD_ARM, "basura", false) == 0); /* sin seq: no hay ack */

	/* e-stop: rueda libre, rechaza gripper y lo reporta */
	run(&r, 1.0, true, 0.3f, 0.0f, 0.0f);
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":8,\"active\":true}", false) > 0 &&
	      strstr(r.ack, "\"ok\""));
	run(&r, 0.1, true, 0.3f, 0.0f, 0.0f);
	CHECK(r.rc.dev[0].duty_mode && r.rc.dev[1].duty_mode);
	CHECK(tel(&r, LLC_TEL_STATUS, buf, sizeof(buf)) > 0 && strstr(buf, "\"state\":\"estop\""));
	CHECK(send(&r, LLC_CMD_GRIPPER,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":9,\"pos\":1,\"max_cur\":1}", false) > 0 &&
	      strstr(r.ack, "\"rejected\""));
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":10,\"active\":false}", false) > 0);
	run(&r, 1.5, true, 0.3f, 0.0f, 0.0f);
	CHECK(NEAR(r.llc.meas.vx, 0.3, 0.03));

	/* odom_reset */
	CHECK(send(&r, LLC_CMD_ODOM_RESET,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":11,\"x\":2.0,\"y\":-1.0,\"yaw\":1.0}",
		   false) > 0 && strstr(r.ack, "\"ok\""));
	CHECK(tel(&r, LLC_TEL_ODOM, buf, sizeof(buf)) > 0);
	CHECK(NEAR(jnum(buf, "x"), 2.0, 1e-3) && NEAR(jnum(buf, "yaw"), 1.0, 1e-3));
	run(&r, 0.5, true, 0.0f, 0.0f, 0.0f);
	CHECK(tel(&r, LLC_TEL_ODOM, buf, sizeof(buf)) > 0 && NEAR(jnum(buf, "yaw"), 1.0, 0.02));
}

/* Paro físico: enclava, se reporta en el bit 7 y no se libera por MQTT mientras
 * el botón siga abierto; soltarlo tampoco reanuda el movimiento por sí solo. */
static void test_estop_hw(void)
{
	rig_t r;
	char buf[400];

	rig_init(&r);
	run(&r, 0.6, false, 0, 0, 0);
	run(&r, 1.0, true, 0.3f, 0.0f, 0.0f);
	CHECK(NEAR(r.llc.meas.vx, 0.3, 0.03));

	r.estop_hw = true;
	run(&r, 0.1, true, 0.3f, 0.0f, 0.0f);
	CHECK(r.llc.estop && (r.llc.faults & AMR_FAULT_ESTOP_HW));
	CHECK(r.rc.dev[0].duty_mode && r.rc.dev[1].duty_mode);
	CHECK(tel(&r, LLC_TEL_STATUS, buf, sizeof(buf)) > 0 && strstr(buf, "\"state\":\"estop\""));
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":20,\"active\":false}", false) > 0 &&
	      strstr(r.ack, "\"rejected\""));

	/* se suelta el botón: sigue en paro hasta el comando explícito */
	r.estop_hw = false;
	run(&r, 0.5, true, 0.3f, 0.0f, 0.0f);
	CHECK(r.llc.estop && !(r.llc.faults & AMR_FAULT_ESTOP_HW));
	CHECK(r.rc.dev[0].duty_mode && r.rc.dev[1].duty_mode);
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":21,\"active\":false}", false) > 0 &&
	      strstr(r.ack, "\"ok\""));
	run(&r, 1.5, true, 0.3f, 0.0f, 0.0f);
	CHECK(!r.llc.estop && NEAR(r.llc.meas.vx, 0.3, 0.03));
}

static void arm_cmd(rig_t *r, unsigned seq, const char *body)
{
	char json[300];

	snprintf(json, sizeof(json), "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":%u,%s}", seq, body);
	send(r, LLC_CMD_ARM, json, false);
}

static void test_arm(void)
{
	rig_t r;
	char buf[600];
	const char *goal = "\"op\":\"goal\",\"pos\":[4096,-2048,1500,-800],"
			   "\"acc\":[200,200,200,200],\"dec\":[200,200,200,200]";

	/* Sin bus CAN: solo disable/clear. */
	rig_init_ex(&r, false);
	run(&r, 0.2, false, 0, 0, 0);
	arm_cmd(&r, 1, "\"op\":\"enable\"");
	CHECK(strstr(r.ack, "\"rejected\"") && strstr(r.ack, "CAN"));
	arm_cmd(&r, 2, "\"op\":\"disable\"");
	CHECK(strstr(r.ack, "\"ok\""));

	rig_init(&r);
	r.gim.m[2].pos = r.gim.m[2].target = 700.0; /* junta 3 fuera de cero al encender */
	run(&r, 0.2, false, 0, 0, 0);
	arm_cmd(&r, 1, goal);
	CHECK(strstr(r.ack, "\"rejected\"") && strstr(r.ack, "enable"));
	arm_cmd(&r, 2, "\"op\":\"enable\"");
	CHECK(strstr(r.ack, "\"ok\""));
	run(&r, 0.2, false, 0, 0, 0);
	CHECK(tel(&r, LLC_TEL_ARM, buf, sizeof(buf)) > 0);
	CHECK(strstr(buf, "\"enabled\":true") && strstr(buf, "\"pos\":[0,0,700,0]"));
	CHECK(strstr(buf, "\"in_pos\":[true,true,true,true]"));

	arm_cmd(&r, 3, goal);
	CHECK(strstr(r.ack, "\"ok\""));
	run(&r, 0.3, false, 0, 0, 0);
	CHECK(tel(&r, LLC_TEL_ARM, buf, sizeof(buf)) > 0 && strstr(buf, "false"));
	run(&r, 3.0, false, 0, 0, 0);
	CHECK(tel(&r, LLC_TEL_ARM, buf, sizeof(buf)) > 0);
	CHECK(strstr(buf, "\"pos\":[4096,-2048,1500,-800]") != NULL);
	CHECK(strstr(buf, "\"in_pos\":[true,true,true,true]") && strstr(buf, "\"fault\":[0,0,0,0]"));

	/* Fuera de los límites del MCU. */
	arm_cmd(&r, 4, "\"op\":\"goal\",\"pos\":[0,0,99999,0],"
		       "\"acc\":[100,100,100,100],\"dec\":[100,100,100,100]");
	CHECK(strstr(r.ack, "\"rejected\"") && strstr(r.ack, "limites"));

	/* E-stop durante un movimiento: el brazo frena y se queda (no se suelta). */
	arm_cmd(&r, 5, "\"op\":\"home\"");
	CHECK(strstr(r.ack, "\"ok\""));
	run(&r, 0.4, false, 0, 0, 0);
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":20,\"active\":true}", false) > 0);
	run(&r, 2.0, false, 0, 0, 0);
	CHECK(r.gim.m[0].enabled && fabs(r.gim.m[0].pos) > 500.0); /* no llegó a home */
	arm_cmd(&r, 6, "\"op\":\"home\"");
	CHECK(strstr(r.ack, "\"rejected\"") && strstr(r.ack, "e-stop"));
	CHECK(send(&r, LLC_CMD_ESTOP,
		   "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":21,\"active\":false}", false) > 0);

	/* Motor sin respuesta: bit 3 (CAN) en llc/status y fault 255 en llc/arm. */
	r.gim.m[1].online = false;
	run(&r, 0.2, false, 0, 0, 0);
	CHECK(r.llc.faults & AMR_FAULT_CAN_BUS);
	CHECK(tel(&r, LLC_TEL_ARM, buf, sizeof(buf)) > 0 && strstr(buf, "\"fault\":[0,255,0,0]"));
	r.gim.m[1].online = true;
	r.gim.m[3].fault = 0x10;
	run(&r, 0.7, false, 0, 0, 0);
	CHECK(!(r.llc.faults & AMR_FAULT_CAN_BUS) && (r.llc.faults & AMR_FAULT_ARM_MOTOR));
	arm_cmd(&r, 7, "\"op\":\"clear\"");
	run(&r, 0.7, false, 0, 0, 0);
	CHECK(!(r.llc.faults & AMR_FAULT_ARM_MOTOR));
}

static void test_roboclaw_fault(void)
{
	rig_t r;
	char buf[400];

	rig_init(&r);
	run(&r, 0.5, true, 0.1f, 0.0f, 0.0f);
	r.rc.dev[1].present = false;
	run(&r, 0.2, true, 0.1f, 0.0f, 0.0f);
	CHECK(r.llc.faults & AMR_FAULT_ROBOCLAW_129);
	CHECK(tel(&r, LLC_TEL_STATUS, buf, sizeof(buf)) > 0 && strstr(buf, "\"state\":\"fault\""));
	CHECK(r.llc.rc_err[1] > 0 && r.llc.rc_err[0] == 0);
	CHECK(tel(&r, LLC_TEL_WHEELS, buf, sizeof(buf)) > 0 && strstr(buf, "\"rc_err\":[0,"));
	r.rc.dev[1].present = true;
	run(&r, 0.2, true, 0.1f, 0.0f, 0.0f);
	CHECK((r.llc.faults & AMR_FAULT_ROBOCLAW_129) == 0);
}

static void test_imu_reset_and_loss(void)
{
	rig_t r;

	rig_init(&r);
	run(&r, 0.5, false, 0, 0, 0);
	CHECK((r.llc.faults & AMR_FAULT_IMU) == 0);
	bno_sim_reset(&r.bno);               /* brown-out del sensor */
	run(&r, 0.3, false, 0, 0, 0);
	CHECK(r.imu.bno.resets >= 1 && r.imu.configured);
	CHECK((r.llc.faults & AMR_FAULT_IMU) == 0);
	r.bno.present = false;               /* cable suelto */
	run(&r, 0.5, false, 0, 0, 0);
	CHECK(r.llc.faults & AMR_FAULT_IMU);
	/* sin IMU la odometría sigue con ruedas */
	run(&r, 1.0, true, 0.2f, 0.0f, 0.0f);
	CHECK(NEAR(r.llc.ekf.s[AMR_EKF_VX], 0.2, 0.02));
	r.bno.present = true;
	bno_sim_reset(&r.bno);
	run(&r, 1.5, false, 0, 0, 0);        /* reintento de configuración cada 1 s */
	CHECK((r.llc.faults & AMR_FAULT_IMU) == 0);
}

static void grip_cmd(rig_t *r, unsigned seq, float pos, float max_cur)
{
	char json[160];

	snprintf(json, sizeof(json),
		 "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":%u,\"pos\":%.3f,\"max_cur\":%.3f}", seq,
		 (double)pos, (double)max_cur);
	CHECK(send(r, LLC_CMD_GRIPPER, json, false) > 0 && strstr(r->ack, "\"ok\""));
}

static void test_gripper(void)
{
	rig_t r;
	char buf[600];

	rig_init(&r);
	r.object_at = 0.6f;                  /* pieza entre los dedos */
	run(&r, 0.6, false, 0, 0, 0);        /* calibra el cero del ACS712 (driver dormido) */
	CHECK(r.llc.grip.zero_ready && NEAR(r.llc.grip.zero_mv, zero_mv, 10.0));
	CHECK(r.grip_sleep);
	grip_cmd(&r, 1, 1.0f, 1.0f);
	run(&r, 2.0, false, 0, 0, 0);
	CHECK(r.llc.grip.grasped && r.llc.grip.state == GRIP_HOLDING);
	CHECK(NEAR(r.llc.grip.pos, 0.6, 0.1)); /* estimada por tiempo: se trabó sobre la pieza */
	CHECK(NEAR(r.grip_true, 0.6, 0.01));
	/* control de corriente: sostiene con |I| = max_cur (duty ~ 1.0 / 2.5) */
	CHECK(NEAR(r.llc.grip.cur_a, 1.0, 0.1) && NEAR(r.grip_duty, 0.4, 0.05) && !r.grip_sleep);
	CHECK(tel(&r, LLC_TEL_ARM, buf, sizeof(buf)) > 0 && strstr(buf, "\"grasped\":true"));
	grip_cmd(&r, 2, 1.0f, 0.5f);         /* menos fuerza sobre la misma pieza */
	run(&r, 1.5, false, 0, 0, 0);
	CHECK(r.llc.grip.grasped && NEAR(r.llc.grip.cur_a, 0.5, 0.1));

	/* e-stop mientras sostiene: no suelta la pieza */
	CHECK(send(&r, LLC_CMD_ESTOP, "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":60,\"active\":true}", false) > 0);
	run(&r, 0.5, false, 0, 0, 0);
	CHECK(r.llc.grip.grasped && r.llc.grip.state == GRIP_HOLDING);
	CHECK(send(&r, LLC_CMD_ESTOP, "{\"v\":1,\"boot_id\":\"" BOOT "\",\"seq\":61,\"active\":false}", false) > 0);

	grip_cmd(&r, 3, 0.0f, 1.0f);         /* abrir suelta y busca el tope */
	CHECK(!r.llc.grip.grasped);
	run(&r, 2.5, false, 0, 0, 0);
	CHECK(r.llc.grip.state == GRIP_IDLE && r.grip_sleep);
	CHECK(r.grip_true <= 1e-3f && r.llc.grip.pos == 0.0f);

	r.object_at = -1.0f;                 /* cerrar en el aire no es agarre */
	grip_cmd(&r, 4, 1.0f, 1.0f);
	run(&r, 2.5, false, 0, 0, 0);
	CHECK(!r.llc.grip.grasped && r.llc.grip.pos == 1.0f && r.grip_sleep);

	/* apertura parcial: se detiene a media carrera (estimada) */
	grip_cmd(&r, 5, 0.5f, 1.0f);
	run(&r, 2.0, false, 0, 0, 0);
	CHECK(r.llc.grip.state == GRIP_IDLE && NEAR(r.grip_true, 0.5, 0.05));

	/* pieza dura con max_cur alto: supera el límite duro, falla y abre un poco */
	grip_cmd(&r, 6, 0.0f, 1.0f);
	run(&r, 2.5, false, 0, 0, 0);
	r.object_at = 0.5f;
	r.stall_a = 4.0f;
	grip_cmd(&r, 7, 1.0f, 5.0f);
	run(&r, 2.5, false, 0, 0, 0);
	CHECK(r.llc.grip.overcurrent && (r.llc.faults & AMR_FAULT_GRIPPER_OVERCURRENT));
	CHECK(!r.llc.grip.grasped && r.grip_true < 0.5f && r.grip_sleep);

	/* nFAULT del DRV8833: se detiene y lo reporta */
	r.stall_a = 2.5f;
	grip_cmd(&r, 8, 1.0f, 1.0f);
	CHECK(!r.llc.grip.overcurrent);      /* un comando nuevo borra la falla */
	run(&r, 0.2, false, 0, 0, 0);
	r.drv_fault = true;
	run(&r, 0.1, false, 0, 0, 0);
	CHECK(r.llc.grip.driver_fault && r.grip_sleep &&
	      (r.llc.faults & AMR_FAULT_GRIPPER_OVERCURRENT));
	r.drv_fault = false;
}

/* El ACS712 se alimenta después que la Tlaloc: el cero del arranque sale en 0 mV y el
 * gripper reportaba ~13 A en reposo. Con el driver dormido el cero se corrige solo. */
static void test_gripper_zero_tracking(void)
{
	rig_t r;

	rig_init(&r);
	r.sensor_off = true;
	run(&r, 0.6, false, 0, 0, 0);
	CHECK(r.llc.grip.zero_ready && r.llc.grip.zero_mv < 10.0f);
	r.sensor_off = false;                /* llegan los 5 V del sensor */
	run(&r, 0.3, false, 0, 0, 0);
	CHECK(r.llc.grip.cur_a > 5.0f);      /* el cero aún está mal */
	run(&r, 3.0, false, 0, 0, 0);
	CHECK(NEAR(r.llc.grip.zero_mv, zero_mv, 5.0) && r.llc.grip.cur_a < 0.05f);
	/* moviendo no se toca el cero (hay corriente real) */
	grip_cmd(&r, 1, 1.0f, 1.0f);
	run(&r, 0.5, false, 0, 0, 0);
	CHECK(r.llc.grip.state == GRIP_CLOSING && NEAR(r.llc.grip.zero_mv, zero_mv, 5.0));
	CHECK(r.llc.grip.cur_a > 0.1f);
}

/* Arranque con el gripper casi cerrado (posición desconocida): la primera apertura
 * empuja hasta el tope; y un pulso suelto de nFAULT (< drv_fault_ms) no lo detiene. */
static void test_gripper_homing_and_nfault_glitch(void)
{
	rig_t r;

	rig_init(&r);
	r.grip_true = 0.8f;
	run(&r, 0.6, false, 0, 0, 0);
	CHECK(!r.llc.grip.homed);
	grip_cmd(&r, 1, 0.0f, 1.0f);
	run(&r, 2.5, false, 0, 0, 0);
	CHECK(r.grip_true <= 1e-3f && r.llc.grip.homed && r.llc.grip.state == GRIP_IDLE);

	grip_cmd(&r, 2, 1.0f, 1.0f);
	run(&r, 0.3, false, 0, 0, 0);
	r.drv_fault = true;                  /* pulso de 20 ms */
	run(&r, 0.02, false, 0, 0, 0);
	r.drv_fault = false;
	run(&r, 0.1, false, 0, 0, 0);
	CHECK(!r.llc.grip.driver_fault && r.llc.grip.state == GRIP_CLOSING);
	CHECK(!(r.llc.faults & AMR_FAULT_GRIPPER_OVERCURRENT));
	r.drv_fault = true;                  /* VM ausente: nFAULT fijo */
	run(&r, 0.1, false, 0, 0, 0);
	CHECK(r.llc.grip.driver_fault && r.grip_sleep);
	r.drv_fault = false;
}

static void test_battery_critical(void)
{
	rig_t r;

	rig_init(&r);
	r.rc.dev[0].battery_v = 19.5f;
	r.rc.dev[1].battery_v = 19.5f;
	run(&r, 1.5, false, 0, 0, 0);
	CHECK(r.llc.faults & AMR_FAULT_BATTERY_CRITICAL);
	CHECK(NEAR(r.llc.batt_v, 19.5, 0.11));
}

int main(void)
{
	test_boot_and_drive();
	test_watchdog_and_invalid();
	test_commands_and_acks();
	test_estop_hw();
	test_roboclaw_fault();
	test_arm();
	test_imu_reset_and_loss();
	test_gripper();
	test_gripper_zero_tracking();
	test_gripper_homing_and_nfault_glitch();
	test_battery_critical();
	if (failures == 0) {
		printf("tlaloc_llc: PASSED (0 failures)\n");
		return 0;
	}
	printf("tlaloc_llc: FAILED (%d failures)\n", failures);
	return 1;
}
