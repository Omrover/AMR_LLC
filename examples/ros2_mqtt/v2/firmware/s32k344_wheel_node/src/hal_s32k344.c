/*
 * S32K344 hardware layer (MR-CANHUBK3 or any board with the same overlay):
 *  - encoder: alias qdec0, NXP eMIOS + LCU + TRGMUX quadrature decoder
 *  - motors : zephyr,user pwms[0..3] = left IN1/IN2, right IN1/IN2 (eMIOS)
 * Pins and calibration live in boards/<board>.overlay, not in this file.
 */
#include "hal.h"
#include "lab_logic.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hal, LOG_LEVEL_INF);

#define QDEC_NODE DT_ALIAS(qdec0)
#define USER_NODE DT_PATH(zephyr_user)

/* The driver converts with integer division, so do the same here. */
#define TICKS_PER_REV (DT_PROP(QDEC_NODE, micro_ticks_per_rev) / 1000000)

static const struct device *const qdec = DEVICE_DT_GET(QDEC_NODE);
static const struct pwm_dt_spec motor[4] = {
	PWM_DT_SPEC_GET_BY_NAME(USER_NODE, left_in1),
	PWM_DT_SPEC_GET_BY_NAME(USER_NODE, left_in2),
	PWM_DT_SPEC_GET_BY_NAME(USER_NODE, right_in1),
	PWM_DT_SPEC_GET_BY_NAME(USER_NODE, right_in2),
};

int hal_init(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(motor); i++) {
		if (!pwm_is_ready_dt(&motor[i])) {
			LOG_ERR("PWM %u not ready", (unsigned int)i);
			return -ENODEV;
		}
		pwm_set_pulse_dt(&motor[i], 0);
	}
	if (!device_is_ready(qdec)) {
		LOG_ERR("qdec0 not ready");
		return -ENODEV;
	}
	LOG_INF("encoder %d ticks/rev, PWM period %u ns", TICKS_PER_REV,
		motor[0].period);
	return 0;
}

int hal_read_rotation(double *rotation_rad)
{
	struct sensor_value val;
	int rc = sensor_sample_fetch(qdec);

	if (rc == 0) {
		rc = sensor_channel_get(qdec, SENSOR_CHAN_ROTATION, &val);
	}
	if (rc == 0) {
		*rotation_rad = sensor_value_to_double(&val);
	}
	return rc;
}

double hal_ticks_per_rev(void)
{
	return (double)TICKS_PER_REV;
}

static int set_wheel(const struct pwm_dt_spec *in1,
		     const struct pwm_dt_spec *in2, float duty)
{
	uint32_t p1, p2;

	student_hbridge_pulses(duty, in1->period, &p1, &p2);
	if (p1 != 0U && p2 != 0U) {
		p1 = p2 = 0U; /* never drive both bridge inputs */
	}
	/* Lower the inactive input first so both are never high together. */
	int rc = pwm_set_pulse_dt(p1 ? in2 : in1, 0);

	rc |= pwm_set_pulse_dt(p1 ? in1 : in2, p1 ? p1 : p2);
	return rc;
}

int hal_set_wheels(float left, float right)
{
	return set_wheel(&motor[0], &motor[1], left) |
	       set_wheel(&motor[2], &motor[3], right);
}
