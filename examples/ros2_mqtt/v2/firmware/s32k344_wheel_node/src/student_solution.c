/* Reference solution, selected with CONFIG_LAB_USE_SOLUTION=y. */
#include "lab_logic.h"

#include <math.h>

int64_t student_ticks_from_rotation(double rotation_rad, double ticks_per_rev)
{
	return llround(rotation_rad * ticks_per_rev / LAB_TWO_PI);
}

void student_hbridge_pulses(float duty, uint32_t period_ns,
			    uint32_t *in1_ns, uint32_t *in2_ns)
{
	*in1_ns = 0U;
	*in2_ns = 0U;
	if (!isfinite(duty) || fabsf(duty) > 1.0f) {
		return;
	}
	uint32_t pulse = (uint32_t)lroundf(fabsf(duty) * (float)period_ns);

	if (duty > 0.0f) {
		*in1_ns = pulse;
	} else if (duty < 0.0f) {
		*in2_ns = pulse;
	}
}
