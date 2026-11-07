/*
 * STUDENT FILE: complete both functions, then run tests/run_host_tests.sh.
 * The firmware prints "student self-test: PASS" on the serial console at boot
 * once they are correct. Until then the encoder reports 0 ticks and the motor
 * inputs stay low (safe default).
 */
#include "lab_logic.h"

#include <math.h>

int64_t student_ticks_from_rotation(double rotation_rad, double ticks_per_rev)
{
	/*
	 * The S32K344 qdec driver returns the cumulative wheel angle in radians:
	 *   rotation_rad = ticks * 2*pi / ticks_per_rev
	 * Invert that and round to the nearest integer (use llround and LAB_TWO_PI).
	 */
	/* ## TODO:: add code here */
	(void)rotation_rad;
	(void)ticks_per_rev;
	return 0;
}

void student_hbridge_pulses(float duty, uint32_t period_ns,
			    uint32_t *in1_ns, uint32_t *in2_ns)
{
	/*
	 * 2-input H-bridge, sign-magnitude drive:
	 *   duty > 0  -> IN1 = |duty| * period, IN2 = 0   (forward)
	 *   duty < 0  -> IN1 = 0, IN2 = |duty| * period   (reverse)
	 *   duty == 0, NaN/inf, or |duty| > 1 -> both 0   (coast, safe)
	 * Never set both inputs high at the same time.
	 */
	*in1_ns = 0U;
	*in2_ns = 0U;
	/* ## TODO:: add code here */
	(void)duty;
	(void)period_ns;
}
