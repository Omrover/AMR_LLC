/*
 * native_sim hardware model: lets the SAME firmware run on the NUC/laptop
 * against a real Mosquitto broker when no S32K344 board is available.
 * The left wheel turns at 1200 ticks/s per unit duty (0.1 -> 120 ticks/s),
 * or at a constant 120 ticks/s with CONFIG_LAB_SIM_HAND_SPIN ("hand spin").
 */
#include "hal.h"
#include "lab_logic.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hal_sim, LOG_LEVEL_INF);

#define SIM_TICKS_PER_REV 600.0
#define SIM_PERIOD_NS 1000000U

static struct k_spinlock lock;
static double ticks;
static double ticks_per_s;
static int64_t last_ms;

static void integrate(void)
{
	int64_t now = k_uptime_get();

	ticks += ticks_per_s * (double)(now - last_ms) / 1000.0;
	last_ms = now;
}

int hal_init(void)
{
	last_ms = k_uptime_get();
	ticks_per_s = IS_ENABLED(CONFIG_LAB_SIM_HAND_SPIN) ? 120.0 : 0.0;
	LOG_INF("simulated encoder %d ticks/rev%s", (int)SIM_TICKS_PER_REV,
		IS_ENABLED(CONFIG_LAB_SIM_HAND_SPIN) ? ", hand spin 120 ticks/s" : "");
	return 0;
}

int hal_read_rotation(double *rotation_rad)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	integrate();
	*rotation_rad = ticks * LAB_TWO_PI / SIM_TICKS_PER_REV;
	k_spin_unlock(&lock, key);
	return 0;
}

double hal_ticks_per_rev(void)
{
	return SIM_TICKS_PER_REV;
}

int hal_set_wheels(float left, float right)
{
	uint32_t in1, in2;

	ARG_UNUSED(right); /* one encoder, on the left wheel */
	/* Use the student's H-bridge mapping so the simulator exercises it too. */
	student_hbridge_pulses(left, SIM_PERIOD_NS, &in1, &in2);
	if (IS_ENABLED(CONFIG_LAB_SIM_HAND_SPIN)) {
		return 0;
	}
	k_spinlock_key_t key = k_spin_lock(&lock);

	integrate();
	ticks_per_s = 1200.0 * ((double)in1 - (double)in2) / SIM_PERIOD_NS;
	k_spin_unlock(&lock, key);
	return 0;
}
