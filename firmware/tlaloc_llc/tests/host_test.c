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
	llc_imu_t imu;
	llc_t llc;
	uint32_t now;
	/* io falso: servo + ACS712 */
	uint32_t servo_us;
	uint32_t servo_writes;
	float object_at;      /* posición del gripper donde toca la pieza (<0: sin pieza) */
	float stall_a;        /* corriente al apretar la pieza */
	bool adc_ok;
	/* verdad del terreno */
	double x, y, yaw;
	uint32_t next_vel_ms;
	uint32_t seq;
	char ack[256];
	int ack_len;
} rig_t;

static const float zero_mv = 1650.0f;   /* 2.5 V del ACS712 a través del divisor 2/3 */

static int io_servo(void *ctx, uint32_t us)
{
	rig_t *r = ctx;

	r->servo_us = us;
	r->servo_writes++;
	return 0;
}

static int io_adc(void *ctx, float *mv)
{
	rig_t *r = ctx;
	float amps = 0.05f;   /* consumo en vacío del servo */

	if (!r->adc_ok) {
		return -5;
	}
	if (r->object_at >= 0.0f && r->llc.grip.pos >= r->object_at) {
		amps = r->stall_a;
	}
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
	r->stall_a = 1.5f;
	r->adc_ok = true;
	rc_sim_init(&r->rc, 128, 129);
	bno_sim_init(&r->bno);
	rt = rc_sim_transport(&r->rc);
	bt = bno_sim_transport(&r->bno);
	gim_sim_init(&r->gim);
	gt = gim_sim_transport(&r->gim);
	io.servo_set_us = io_servo;
	io.adc_read_mv = io_adc;
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

		upd = llc_imu_service(&r->imu, r->now, 4);
		if (upd > 0) {
			llc_imu_sample(&r->llc, &r->imu.bno.data, (uint32_t)upd, r->now);
		}
		if (stream && (int32_t)(r->now - r->next_vel_ms) >= 0) {
			send_vel(r, vx, vy, wz);
			r->next_vel_ms = r->now + 50;
		}
		if (r->now % 20 == 0) {
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
	CHECK(r.llc.servo_us == 1000);   /* gripper abierto */

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
	run(&r, 0.6, false, 0, 0, 0);        /* calibra el cero del ACS712 */
	CHECK(r.llc.grip.zero_ready && NEAR(r.llc.grip.zero_mv, zero_mv, 10.0)); /* incluye el consumo en reposo */
	grip_cmd(&r, 1, 1.0f, 1.0f);
	run(&r, 1.5, false, 0, 0, 0);
	CHECK(r.llc.grip.grasped);
	CHECK(NEAR(r.llc.grip.pos, 0.6, 0.06)); /* se detuvo sobre la pieza */
	CHECK(tel(&r, LLC_TEL_ARM, buf, sizeof(buf)) > 0 && strstr(buf, "\"grasped\":true"));
	CHECK(r.servo_us > 1500 && r.servo_us < 1700);
	grip_cmd(&r, 2, 0.0f, 1.0f);         /* abrir suelta */
	CHECK(!r.llc.grip.grasped);
	run(&r, 1.5, false, 0, 0, 0);
	CHECK(r.servo_us == 1000);

	r.object_at = -1.0f;                 /* cerrar en el aire no es agarre */
	grip_cmd(&r, 3, 1.0f, 1.0f);
	run(&r, 1.5, false, 0, 0, 0);
	CHECK(!r.llc.grip.grasped && r.llc.grip.pos == 1.0f);

	/* pieza dura con max_cur alto: supera el límite duro y abre un poco */
	grip_cmd(&r, 4, 0.0f, 1.0f);
	run(&r, 1.5, false, 0, 0, 0);
	r.object_at = 0.5f;
	r.stall_a = 4.0f;
	grip_cmd(&r, 5, 1.0f, 5.0f);
	run(&r, 1.5, false, 0, 0, 0);
	CHECK(r.llc.grip.overcurrent && (r.llc.faults & AMR_FAULT_GRIPPER_OVERCURRENT));
	run(&r, 1.5, false, 0, 0, 0);          /* sigue abriendo mientras la corriente no baje */
	CHECK(r.llc.grip.pos < 0.5f && !r.llc.grip.grasped);
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
	test_roboclaw_fault();
	test_arm();
	test_imu_reset_and_loss();
	test_gripper();
	test_battery_critical();
	if (failures == 0) {
		printf("tlaloc_llc: PASSED (0 failures)\n");
		return 0;
	}
	printf("tlaloc_llc: FAILED (%d failures)\n", failures);
	return 1;
}
