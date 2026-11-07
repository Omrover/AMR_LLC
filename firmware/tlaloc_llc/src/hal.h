/*
 * Capa de hardware de tlaloc_llc. Dos implementaciones:
 *   hal_tlaloc.c: Zephyr en la Tlaloc (LPUART3, LPI2C0, eMIOS1, ADC0, FS26)
 *   hal_sim.c:    native_sim, con los simuladores de RoboClaw y BNO080
 * Todo lo demás (llc.c y las librerías de lib/) es igual en ambas.
 */
#ifndef TLALOC_HAL_H
#define TLALOC_HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "bno08x.h"
#include "llc.h"
#include "roboclaw.h"

int hal_init(void);
rc_transport_t hal_roboclaw_transport(void);
bno_transport_t hal_imu_transport(void);
llc_io_t hal_io(void);
/* Bloquea hasta la interrupción de la BNO080 o hasta timeout_ms. */
void hal_imu_wait(int32_t timeout_ms);
/* Arranca el watchdog externo (FS26). 0 si quedó activo, <0 si no hay. */
int hal_wdt_start(void);
/* Llamar desde el lazo de control cada ~20 ms; el HAL respeta la ventana. */
void hal_wdt_kick(uint32_t now_ms);

#endif /* TLALOC_HAL_H */
