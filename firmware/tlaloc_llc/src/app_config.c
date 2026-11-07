/* Configuración del núcleo a partir de Kconfig (prj.conf / boards/*.conf). */
#include "app_config.h"

void app_config(llc_cfg_t *cfg)
{
	llc_default_cfg(cfg);

	cfg->drive.counts_per_rev = (float)CONFIG_TLALOC_COUNTS_PER_REV;
	/* QPPS máximo de las RoboClaw -> límite por rueda, con 3 % de margen */
	cfg->drive.max_wheel_rad_s =
		0.97f * (float)CONFIG_TLALOC_MAX_QPPS * 6.28318530718f / cfg->drive.counts_per_rev;
	cfg->drive.map[0].sign = (int8_t)CONFIG_TLALOC_SIGN_WHEEL1_FL;
	cfg->drive.map[1].sign = (int8_t)CONFIG_TLALOC_SIGN_WHEEL2_RL;
	cfg->drive.map[2].sign = (int8_t)CONFIG_TLALOC_SIGN_WHEEL3_RR;
	cfg->drive.map[3].sign = (int8_t)CONFIG_TLALOC_SIGN_WHEEL4_FR;

	cfg->imu_z_sign = IS_ENABLED(CONFIG_TLALOC_IMU_Z_DOWN) ? -1.0f : 1.0f;

	cfg->grip.pulse_open_us = CONFIG_TLALOC_GRIP_PULSE_OPEN_US;
	cfg->grip.pulse_closed_us = CONFIG_TLALOC_GRIP_PULSE_CLOSED_US;
	cfg->grip.mv_per_a = (float)CONFIG_TLALOC_ACS712_MV_PER_A_X10 / 10.0f;
}
