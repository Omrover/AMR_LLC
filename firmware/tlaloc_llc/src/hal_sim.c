/*
 * HAL de native_sim: el firmware corre como proceso Linux contra los mismos
 * simuladores de bytes que usan las pruebas en host (RoboClaw Packet Serial y
 * BNO080 SHTP). Un hilo avanza la física a 200 Hz: velocidad real de las
 * ruedas -> cinemática directa -> pose real -> reportes de la IMU.
 * Así el bridge de la NUC ve un MCU completo sin hardware.
 */
#include <errno.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_config.h"
#include "bno08x_sim.h"
#include "hal.h"
#include "omni.h"
#include "roboclaw_sim.h"

LOG_MODULE_REGISTER(hal_sim, LOG_LEVEL_INF);

#define SIM_PERIOD_MS 5

static K_MUTEX_DEFINE(sim_lock);
static rc_sim_t rc;
static bno_sim_t bno;
static rc_transport_t rc_inner;
static bno_transport_t bno_inner;
static amr_drive_cfg_t geo;
static amr_omni_t kin;
static double yaw;
static float grip_pos_us = 1000.0f;
static K_SEM_DEFINE(imu_sem, 0, 1);

/* --------------------------------------------------------------- RoboClaw */

static int sim_rc_write(void *ctx, const uint8_t *buf, size_t n)
{
	int r;

	k_mutex_lock(&sim_lock, K_FOREVER);
	r = rc_inner.write(ctx, buf, n);
	k_mutex_unlock(&sim_lock);
	return r;
}

static int sim_rc_read(void *ctx, uint8_t *buf, size_t n, uint32_t timeout_ms)
{
	int r;

	k_mutex_lock(&sim_lock, K_FOREVER);
	r = rc_inner.read(ctx, buf, n, timeout_ms);
	k_mutex_unlock(&sim_lock);
	if (r != 0) {
		k_msleep(timeout_ms); /* un timeout real también consume el tiempo */
	}
	return r;
}

static void sim_rc_flush(void *ctx)
{
	k_mutex_lock(&sim_lock, K_FOREVER);
	if (rc_inner.flush_rx != NULL) {
		rc_inner.flush_rx(ctx);
	}
	k_mutex_unlock(&sim_lock);
}

rc_transport_t hal_roboclaw_transport(void)
{
	rc_transport_t t = {sim_rc_write, sim_rc_read, sim_rc_flush, rc_inner.ctx};

	return t;
}

/* ----------------------------------------------------------------- BNO080 */

static int sim_imu_write(void *ctx, const uint8_t *buf, size_t n)
{
	int r;

	k_mutex_lock(&sim_lock, K_FOREVER);
	r = bno_inner.write(ctx, buf, n);
	k_mutex_unlock(&sim_lock);
	return r;
}

static int sim_imu_read(void *ctx, uint8_t *buf, size_t n)
{
	int r;

	k_mutex_lock(&sim_lock, K_FOREVER);
	r = bno_inner.read(ctx, buf, n);
	k_mutex_unlock(&sim_lock);
	return r;
}

bno_transport_t hal_imu_transport(void)
{
	bno_transport_t t = {sim_imu_write, sim_imu_read, bno_inner.ctx};

	return t;
}

void hal_imu_wait(int32_t timeout_ms)
{
	(void)k_sem_take(&imu_sem, K_MSEC(timeout_ms));
}

/* --------------------------------------------------------- servo y ACS712 */

static int sim_servo(void *ctx, uint32_t pulse_us)
{
	ARG_UNUSED(ctx);
	grip_pos_us = (float)pulse_us;
	return 0;
}

static int sim_adc(void *ctx, float *mv)
{
	ARG_UNUSED(ctx);
	/* ACS712 en reposo (0 A) visto a través del divisor: 2.5 V * 2/3. Con
	 * CONFIG_TLALOC_SIM_GRIP_OBJECT el servo "toca" una pieza a media carrera. */
	float amps = 0.05f;

	if (IS_ENABLED(CONFIG_TLALOC_SIM_GRIP_OBJECT) && grip_pos_us >= 1550.0f) {
		amps = 1.5f;
	}
	*mv = 1650.0f + amps * (float)CONFIG_TLALOC_ACS712_MV_PER_A_X10 / 10.0f;
	return 0;
}

llc_io_t hal_io(void)
{
	llc_io_t io = {sim_servo, sim_adc, NULL};

	return io;
}

int hal_wdt_start(void)
{
	return -ENOTSUP;
}

void hal_wdt_kick(uint32_t now_ms)
{
	ARG_UNUSED(now_ms);
}

/* ----------------------------------------------------------------- física */

static void sim_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	const double dt = SIM_PERIOD_MS * 1e-3;

	while (true) {
		float w[AMR_OMNI_WHEELS], vx, vy, wz;

		k_mutex_lock(&sim_lock, K_FOREVER);
		rc_sim_step(&rc, dt);
		for (int i = 0; i < AMR_OMNI_WHEELS; i++) {
			const amr_wheel_map_t *m = &geo.map[i];

			w[i] = (float)(rc.dev[m->rc].speed[m->motor] * m->sign * 6.283185307179586 /
				       geo.counts_per_rev);
		}
		amr_omni_forward(&kin, w, &vx, &vy, &wz);
		yaw += wz * dt;
		bno_sim_set_motion(&bno, (float)yaw, wz, 0.0f, 0.0f);
		bno_sim_step(&bno, dt);
		k_mutex_unlock(&sim_lock);
		k_sem_give(&imu_sem);
		k_msleep(SIM_PERIOD_MS);
	}
}

K_THREAD_DEFINE(sim_tid, 2048, sim_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0,
		K_TICKS_FOREVER);

int hal_init(void)
{
	llc_cfg_t cfg;

	app_config(&cfg); /* misma geometría y signos que el núcleo */
	geo = cfg.drive;
	if (!amr_omni_init(&kin, geo.wheel_radius, geo.center_distance, geo.angles_rad)) {
		return -EINVAL;
	}
	rc_sim_init(&rc, geo.rc_addr[0], geo.rc_addr[1]);
	bno_sim_init(&bno);
	bno.gyro_bias = 0.01f; /* sesgo realista: el EKF debe absorberlo con el yaw */
	rc_inner = rc_sim_transport(&rc);
	bno_inner = bno_sim_transport(&bno);
	k_thread_start(sim_tid);
	LOG_INF("native_sim: 2 RoboClaw y BNO080 simuladas");
	return 0;
}
