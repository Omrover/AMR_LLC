#ifndef TLALOC_APP_CONFIG_H
#define TLALOC_APP_CONFIG_H

#include <zephyr/kernel.h>

#include "llc.h"

/* Valores por defecto de llc + ajustes de Kconfig (signos, cuentas, servo...). */
void app_config(llc_cfg_t *cfg);

#endif /* TLALOC_APP_CONFIG_H */
