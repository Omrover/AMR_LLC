/* Copyright 2023 NXP */
/* License: BSD 3-clause
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/
/* Cableado y comunicación de las prácticas de lectura, velocidad y posición.
 * PTD2 J11-5 -> S1 señal; S2 señal -> PTD3 J11-7; tierra compartida.
 * RoboClaw: dirección 128, 38400 baud, 8N1, Packet Serial de una unidad.
 * UART6 PTA16/15 conecta la consola al USB de OpenSDA.
 * Arrancamos desarmados. No escribimos EEPROM ni reiniciamos el encoder.
 * La E/S de esta placa usa 5V y S2 transmite 3.3V: conviene adaptar ese
 * nivel de recepción para no trabajar con tan poco margen.
 * Referencias: S32K3XXRM tabla 130, S32K312_IOMUX.xlsx y UM12505 tabla 14.
 */
#include "S32K312.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "student_speed_pid.h"
#include "student_speed_pid_selftest.h"
#include "student_position_control.h"
#include "lab_profile.h"

#define RC_ADDRESS 128U
#define RC_GET_VERSION 21U
#define RC_GET_ENCODER_M1 16U
#define RC_GET_SPEED_M1 18U
#define RC_GET_BATTERY 24U
#define UART_ERRORS (0xFU << 16U)
#define RX_FULL (1UL << 21U)
#define TX_EMPTY (1UL << 23U)
#define REG32(address) (*(volatile uint32_t *)(address))
#define DEFAULT_SPEED_CPS 500L
#define PID_PERIOD_MS 50U
#define PID_MAX_INTERVAL_MS 100U

/* Estas variables permiten revisar el estado en Expressions sin leer DATA.
 * Estado: 0 arrancando, 1 correcto, 2 recepción sin respuesta, 3 CRC distinto,
 * 4 fallo UART, 5 cadena larga, 6 transmisión sin respuesta, 7 velocidad inválida,
 * 10 reloj no admitido, 11 reloj sin habilitar, 12 DWT detenido,
 * 13 divisor deshabilitado, 14 error excesivo de baudrate.
 */
volatile uint32_t rc_status, rc_attempts, rc_successes, rc_failures;
volatile uint32_t rc_rx_count, rc_uart_error_flags;
volatile uint32_t rc_crc_calculated, rc_crc_received;
volatile uint32_t core_clock_hz, uart_clock_hz, rc_actual_baud;
volatile uint32_t rc_rx_bytes[64];
volatile char rc_version[64];
volatile int32_t rc_position_m1, rc_speed_m1_cps;
volatile uint32_t rc_battery_mv;
volatile uint32_t rc_encoder_flags, rc_speed_direction;
volatile uint32_t rc_position_status, rc_speed_status, rc_voltage_status;
volatile uint32_t rc_version_status, rc_sample_valid, rc_sample_number;
volatile uint32_t rc_last_command, rc_failed_command;
/* Conservamos el primer fallo de cada episodio antes de consultar voltaje
 * o enviar la parada. Las consultas posteriores no borran esta captura.
 * Motivo: bit 0 = representación de velocidad inválida; bit 1 = dirección fuera de 0/1.
 */
volatile uint32_t rc_speed_fault_events, rc_speed_fault_sample, rc_speed_fault_status;
volatile uint32_t rc_speed_fault_count, rc_speed_fault_bytes[7];
volatile uint32_t rc_speed_fault_crc_calculated, rc_speed_fault_crc_received;
volatile uint32_t rc_speed_fault_uart_flags, rc_speed_fault_reason;
volatile uint32_t rc_speed_fault_raw, rc_speed_fault_direction, rc_speed_fault_armed;
volatile int32_t rc_speed_fault_position, rc_speed_fault_duty;
static uint32_t rc_speed_fault_active, rc_speed_fault_pending;
static uint32_t ticks_per_ms;
volatile uint32_t control_armed, control_mode, rc_write_status, rc_stop_status;
volatile uint32_t pc_rx_faults;
volatile int32_t control_target;
volatile int32_t speed_min_cps = -DEFAULT_SPEED_CPS, speed_max_cps = DEFAULT_SPEED_CPS;
/* La etapa de posición exige cero lógico y ventana explícita antes de armar. */
volatile uint32_t position_window_set;
volatile int32_t position_min, position_max;
volatile int32_t position_origin, position_target, position_error;
volatile uint32_t position_zero_set, position_target_set, position_reached;
volatile int32_t position_kp_milli = 500, position_kd_milli, position_tolerance = 10;
static StudentPositionControl position_control = {.kp = 0.5f, .kd = 0.0f, .tolerance = 10};
static uint32_t sample_time, stop_retry_time;
static uint32_t poll_time, print_time, pid_time;
/* Punto de partida para la práctica; cada montaje requiere su propio ajuste. */
static StudentSpeedPid speed_pid = {.kp = 0.020f, .ki = 0.050f, .kd = 0.0f, .duty_limit = 15.0f};
volatile int32_t pid_kp_milli = 20, pid_ki_milli = 50, pid_kd_milli;
volatile int32_t pid_duty_max_pct = 15;
volatile int32_t pid_reference_cps, pid_error_cps, pid_duty_milli_pct;
volatile int32_t pid_integral_milli_pct, pid_duty_raw;
volatile uint32_t pid_dt_us, pid_steps;
volatile uint32_t pid_selftest_errors;
static uint32_t pc_enabled, stop_pending;
static char pc_ring[128];
static uint32_t pc_head, pc_tail;
static void pc_collect(void);

static void led_init(void)
{
    SIUL2.GPDO29.R = 1U; SIUL2.GPDO30.R = 1U; SIUL2.GPDO31.R = 1U;
    SIUL2.MSCR[29].R = 1UL << 21U;
    SIUL2.MSCR[30].R = 1UL << 21U;
    SIUL2.MSCR[31].R = 1UL << 21U;
    SIUL2.MSCR[51].R = 1UL << 19U; /* SW2 PTB19: presionado = 1 */
}

static void led_result(uint32_t ok)
{
    SIUL2.GPDO29.R = ok ? 1U : 0U;
    SIUL2.GPDO30.R = ok ? 0U : 1U;
    SIUL2.GPDO31.R = 1U;
}

static uint32_t cycles(void) { return REG32(0xE0001004UL); }
static uint32_t expired(uint32_t start, uint32_t ms)
{
    return (uint32_t)(cycles() - start) >= ticks_per_ms * ms;
}
static void delay_ms(uint32_t ms)
{
    uint32_t start = cycles();
    while (!expired(start, ms)) { }
}

static uint32_t clocks_init(void)
{
    uint32_t div, firc_hz, start, probe;
    /* Leemos el reloj real, no el SystemCoreClock de la plantilla.
     * Aquí usamos FIRC y sus divisores; una configuración con PLL se rechaza.
     */
    if (MC_CGM.MUX_0_CSS.B.SELSTAT != 0U || MC_CGM.MUX_0_CSS.B.SWIP)
        return 10U;
    if (!MC_CGM.MUX_0_DC_0.B.DE || !MC_CGM.MUX_0_DC_2.B.DE) return 13U;
    div = CONFIGURATION_GPR.CONFIG_REG_GPR.B.FIRC_DIV_SEL;
    firc_hz = (div == 3U) ? 48000000UL :
              ((div == 2U) ? 3000000UL : 24000000UL);
    core_clock_hz = firc_hz / (MC_CGM.MUX_0_DC_0.B.DIV + 1U);
    /* UART3 y UART6 usan AIPS_SLOW_CLK, no el reloj del núcleo. */
    uart_clock_hz = firc_hz / (MC_CGM.MUX_0_DC_2.B.DIV + 1U);
    ticks_per_ms = core_clock_hz / 1000U;
    REG32(0xE000EDFCUL) |= 1UL << 24U; /* DEMCR.TRCENA */
    REG32(0xE0001000UL) |= 1U;         /* DWT.CTRL.CYCCNTENA */
    probe = cycles();
    for (div = 0U; div < 1000U; ++div) { __asm volatile ("nop"); }
    if (cycles() == probe) return 12U;
    /* Habilitamos UART3 (REQ77) y UART6 (REQ80) en MC_ME. */
    MC_ME.PRTN1_PCONF.B.PCE = 1U;
    MC_ME.PRTN1_COFB2_CLKEN.R |= (1UL << 13U) | (1UL << 16U);
    MC_ME.PRTN1_PUPD.B.PCUD = 1U;
    MC_ME.CTL_KEY.R = 0x5AF0U;
    MC_ME.CTL_KEY.R = 0xA50FU;
    start = cycles();
    while (MC_ME.PRTN1_PUPD.B.PCUD ||
           (MC_ME.PRTN1_COFB2_STAT.R & 0x12000UL) != 0x12000UL) {
        if (expired(start, 100U)) return 11U;
    }
    return 0U;
}

static uint32_t uart_init(volatile struct LPUART_tag *uart, uint32_t baud)
{
    uint32_t osr, sbr, actual, error, best_error = 0xFFFFFFFFUL;
    uint32_t best_osr = 0U, best_sbr = 0U, candidate;
    /* Buscamos OSR/SBR con el menor error: 8N1, sin DMA ni interrupciones. */
    for (osr = 4U; osr <= 32U; ++osr) {
        sbr = uart_clock_hz / (baud * osr);
        for (candidate = sbr; candidate <= sbr + 1U; ++candidate) {
            if (!candidate || candidate > 8191U) continue;
            actual = uart_clock_hz / (osr * candidate);
            error = (actual > baud) ? actual - baud : baud - actual;
            if (error <= best_error) {
                best_error = error; best_osr = osr; best_sbr = candidate;
            }
        }
    }
    if (best_error > baud / 50U) return 14U;
    uart->GLOBAL.B.RST = 1U;
    uart->GLOBAL.B.RST = 0U;
    uart->CTRL.R = 0U;
    uart->BAUD.R = ((best_osr - 1U) << 24U) | best_sbr |
                  ((best_osr < 8U) ? (1UL << 17U) : 0U);
    uart->FIFO.R = (1UL << 14U) | (1UL << 15U); /* Limpiamos las colas. */
    uart->WATER.R = 0U;
    uart->STAT.R = UART_ERRORS;
    uart->CTRL.R = (1UL << 18U) | (1UL << 19U); /* RE, TE */
    if (baud == 38400U) rc_actual_baud = uart_clock_hz / (best_osr * best_sbr);
    return 0U;
}

static int uart_put(volatile struct LPUART_tag *uart, uint8_t byte)
{
    uint32_t start = cycles();
    while (!(uart->STAT.R & TX_EMPTY)) {
        pc_collect();
        if (expired(start, 50U)) return 0;
    }
    uart->DATA.R = byte;
    return 1;
}

static void console(const char *text)
{
    while (*text) {
        if (!uart_put(&LPUART_6, (uint8_t)*text++)) break;
    }
}

static void console_u32(uint32_t value)
{
    char digits[10];
    uint32_t count = 0U;
    do { digits[count++] = (char)('0' + value % 10U); value /= 10U; }
    while (value);
    while (count) (void)uart_put(&LPUART_6, (uint8_t)digits[--count]);
}

static void console_i32(int32_t value)
{
    /* Pasamos a unsigned para imprimir también INT32_MIN sin desbordarlo. */
    if (value < 0) { console("-"); console_u32(0U - (uint32_t)value); }
    else console_u32((uint32_t)value);
}

static int rc_get(uint8_t *byte)
{
    uint32_t stat, start = cycles();
    for (;;) {
        pc_collect();
        stat = LPUART_3.STAT.R;
        if (stat & UART_ERRORS) {
            rc_uart_error_flags = stat & UART_ERRORS;
            LPUART_3.STAT.R = UART_ERRORS;
            return -1;
        }
        if (stat & RX_FULL) {
            *byte = (uint8_t)LPUART_3.DATA.R;
            if (rc_rx_count < 64U) rc_rx_bytes[rc_rx_count] = *byte;
            ++rc_rx_count;
            return 1;
        }
        if (expired(start, control_armed ? 20U : 100U)) return 0;
    }
}

static uint16_t crc_byte(uint16_t crc, uint8_t data)
{
    uint32_t bit;
    crc ^= (uint16_t)data << 8U;
    for (bit = 0U; bit < 8U; ++bit)
        crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    return crc;
}

static void prepare_request(uint8_t command)
{
    uint32_t i;
    rc_last_command = command;
    rc_rx_count = 0U; rc_uart_error_flags = 0U;
    rc_crc_calculated = 0U; rc_crc_received = 0U;
    for (i = 0U; i < 64U; ++i) rc_rx_bytes[i] = 0U;
    LPUART_3.FIFO.R = (1UL << 14U) | (1UL << 15U);
    LPUART_3.STAT.R = UART_ERRORS;
}

static uint32_t read_packet(uint8_t command, uint8_t *payload, uint32_t length)
{
    uint32_t i;
    uint8_t hi, lo;
    int got;
    uint16_t crc = crc_byte(crc_byte(0U, RC_ADDRESS), command);
    prepare_request(command);
    if (!uart_put(&LPUART_3, RC_ADDRESS) ||
        !uart_put(&LPUART_3, command)) return 6U;
    for (i = 0U; i < length; ++i) {
        got = rc_get(&payload[i]);
        if (got != 1) return (got == 0) ? 2U : 4U;
        crc = crc_byte(crc, payload[i]);
    }
    got = rc_get(&hi);
    if (got != 1) return (got == 0) ? 2U : 4U;
    got = rc_get(&lo);
    if (got != 1) return (got == 0) ? 2U : 4U;
    rc_crc_calculated = crc;
    rc_crc_received = ((uint16_t)hi << 8U) | lo;
    return (rc_crc_received == crc) ? 1U : 3U;
}

static uint32_t be32(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 24U) | ((uint32_t)bytes[1] << 16U) |
           ((uint32_t)bytes[2] << 8U) | bytes[3];
}

static int32_t signed_count(uint32_t raw)
{
    return (raw <= 0x7FFFFFFFUL) ? (int32_t)raw : -1 - (int32_t)(~raw);
}

static uint32_t speed_decode_reason(uint32_t raw, uint32_t direction)
{
    /* En reversa admitimos magnitud positiva o complemento a dos.
     * Un valor con signo negativo exige dirección 1. Excluimos INT32_MIN
     * porque su magnitud no cabe en int32 positivo.
     */
    return ((raw == 0x80000000UL ||
             (raw > 0x7FFFFFFFUL && direction != 1U)) ? 1U : 0U) |
           (direction > 1U ? 2U : 0U);
}

static int32_t decoded_speed(uint32_t raw, uint32_t direction)
{
    /* Llamar solo después de validar CRC, dirección y representación.
     * Si los bits ya representan un negativo, no invertimos el signo otra vez.
     */
    if (raw > 0x7FFFFFFFUL) return signed_count(raw);
    return direction ? -(int32_t)raw : (int32_t)raw;
}

static int32_t measured_position(void)
{
    /* Diferencia modular: un cruce del contador int32 no cambia el cero lógico. */
    return LAB_STAGE == LAB_NXP_POSITION ?
        signed_count((uint32_t)rc_position_m1 - (uint32_t)position_origin) : rc_position_m1;
}

static void capture_speed_fault(const uint8_t *payload)
{
    uint32_t i;
    ++rc_speed_fault_events;
    rc_speed_fault_sample = rc_sample_number + 1U;
    rc_speed_fault_status = rc_speed_status;
    rc_speed_fault_count = rc_rx_count;
    for (i = 0U; i < 7U; ++i)
        rc_speed_fault_bytes[i] = i < rc_rx_count ? rc_rx_bytes[i] : 0U;
    rc_speed_fault_crc_calculated = rc_crc_calculated;
    rc_speed_fault_crc_received = rc_crc_received;
    rc_speed_fault_uart_flags = rc_uart_error_flags;
    rc_speed_fault_raw = 0U; rc_speed_fault_direction = 0U; rc_speed_fault_reason = 0U;
    if (rc_rx_count >= 5U) {
        rc_speed_fault_raw = be32(payload);
        rc_speed_fault_direction = payload[4];
        if (rc_speed_status == 7U)
            rc_speed_fault_reason = speed_decode_reason(rc_speed_fault_raw,
                                                        rc_speed_fault_direction);
    }
    rc_speed_fault_armed = control_armed;
    rc_speed_fault_position = rc_position_m1;
    rc_speed_fault_duty = pid_duty_milli_pct;
    rc_speed_fault_pending = 1U;
}

static void console_hex(uint32_t value, uint32_t digits)
{
    static const char hex[] = "0123456789ABCDEF";
    while (digits)
        (void)uart_put(&LPUART_6, (uint8_t)hex[(value >> (--digits * 4U)) & 15U]);
}

static void print_speed_fault(void)
{
    uint32_t i;
    /* Primero desarmamos y solicitamos duty cero; no imprimimos dentro del PID. */
    if (!rc_speed_fault_pending || control_armed) return;
    rc_speed_fault_pending = 0U;
    console("DIAG_VELOCIDAD evento="); console_u32(rc_speed_fault_events);
    console(" muestra="); console_u32(rc_speed_fault_sample);
    console(" comando=18 estado="); console_u32(rc_speed_fault_status);
    console(" motivo_bits="); console_u32(rc_speed_fault_reason);
    console(" recibido="); console_u32(rc_speed_fault_count);
    console(" bytes_hex=");
    for (i = 0U; i < rc_speed_fault_count && i < 7U; ++i) {
        if (i) console(",");
        console_hex(rc_speed_fault_bytes[i], 2U);
    }
    if (!rc_speed_fault_count) console("NA");
    console(" bruto_hex=");
    if (rc_speed_fault_count >= 5U) {
        console_hex(rc_speed_fault_raw, 8U);
        console(" interpretado_int32="); console_i32(signed_count(rc_speed_fault_raw));
        console(" direccion="); console_u32(rc_speed_fault_direction);
    } else console("NA interpretado_int32=NA direccion=NA");
    console(" crc_calculado_hex="); console_hex(rc_speed_fault_crc_calculated, 4U);
    console(" crc_recibido_hex="); console_hex(rc_speed_fault_crc_received, 4U);
    console(" uart_flags_hex="); console_hex(rc_speed_fault_uart_flags, 8U);
    console(" armado_al_fallar="); console_u32(rc_speed_fault_armed);
    console(" encoder_bruto="); console_i32(rc_speed_fault_position);
    console(" duty_milesimas="); console_i32(rc_speed_fault_duty);
    console("\r\n");
}

static void read_telemetry(void)
{
    uint8_t payload[5];
    uint32_t raw_speed;
    rc_sample_valid = 0U;
    rc_failed_command = 0U;
    /* Solo actualizamos la medición si pasó el CRC. Si falla mostramos NA,
     * no cero. Las tres consultas se hacen una tras otra, no simultáneamente.
     */
    rc_position_status = read_packet(RC_GET_ENCODER_M1, payload, 5U);
    if (rc_position_status == 1U) {
        rc_position_m1 = signed_count(be32(payload));
        rc_encoder_flags = payload[4];
    }
    rc_speed_status = read_packet(RC_GET_SPEED_M1, payload, 5U);
    if (rc_speed_status == 1U) {
        raw_speed = be32(payload);
        if (speed_decode_reason(raw_speed, payload[4])) rc_speed_status = 7U;
        else {
            rc_speed_direction = payload[4];
            rc_speed_m1_cps = decoded_speed(raw_speed, payload[4]);
        }
    }
    if (rc_speed_status != 1U) {
        if (!rc_speed_fault_active) capture_speed_fault(payload);
        rc_speed_fault_active = 1U;
    } else rc_speed_fault_active = 0U;
    rc_voltage_status = read_packet(RC_GET_BATTERY, payload, 2U);
    if (rc_voltage_status == 1U)
        rc_battery_mv = (((uint32_t)payload[0] << 8U) | payload[1]) * 100U;
    rc_status = 1U;
    if (rc_position_status != 1U) {
        rc_status = rc_position_status; rc_failed_command = RC_GET_ENCODER_M1;
    } else if (rc_speed_status != 1U) {
        rc_status = rc_speed_status; rc_failed_command = RC_GET_SPEED_M1;
    } else if (rc_voltage_status != 1U) {
        rc_status = rc_voltage_status; rc_failed_command = RC_GET_BATTERY;
    }
    rc_sample_valid = (rc_status == 1U);
    ++rc_sample_number;
}

static void print_telemetry(void)
{
    console("n="); console_u32(rc_sample_number);
    console(" pos=");
    if (rc_position_status == 1U) console_i32(measured_position()); else console("NA");
    console(" conteos velocidad=");
    if (rc_speed_status == 1U) console_i32(rc_speed_m1_cps); else console("NA");
    console(" conteos/s voltaje=");
    if (rc_voltage_status == 1U) {
        console_u32(rc_battery_mv / 1000U); console(".");
        console_u32((rc_battery_mv % 1000U) / 100U);
    } else console("NA");
    console(" V estado(pos,vel,V)=");
    console_u32(rc_position_status); console(",");
    console_u32(rc_speed_status); console(","); console_u32(rc_voltage_status);
    console(" consigna="); console_i32(control_target);
    console(" ref="); console_i32(pid_reference_cps);
    console(" error="); console_i32(pid_error_cps);
    console(" duty_milesimas=");
    if (LAB_STAGE >= LAB_NXP_PID) console_i32(pid_duty_milli_pct); else console("NA");
    console(" intervalo_us="); console_u32(pid_dt_us);
    console(" armado="); console_u32(control_armed);
    console(" duty_maximo=");
    if (LAB_STAGE >= LAB_NXP_PID) console_i32(pid_duty_max_pct); else console("NA");
    if (LAB_STAGE == LAB_NXP_POSITION) {
        console(" destino_pos=");
        if (position_target_set) console_i32(position_target); else console("NA");
        console(" error_pos=");
        if (position_target_set && rc_position_status == 1U) console_i32(position_error); else console("NA");
        console(" en_posicion="); console_u32(position_reached);
        console(" encoder_bruto=");
        if (rc_position_status == 1U) console_i32(rc_position_m1); else console("NA");
    }
    console("\r\n");
}

static uint32_t read_version(void)
{
    uint32_t i;
    uint8_t byte, hi, lo;
    int got;
    uint16_t crc = crc_byte(crc_byte(0U, RC_ADDRESS), RC_GET_VERSION);
    prepare_request(RC_GET_VERSION);
    for (i = 0U; i < 64U; ++i) rc_version[i] = 0;
    /* Para leer enviamos dirección y comando, sin CRC de salida.
     * El CRC recibido incluye también el NUL que termina la versión.
     */
    if (!uart_put(&LPUART_3, RC_ADDRESS) ||
        !uart_put(&LPUART_3, RC_GET_VERSION)) return 6U;
    for (i = 0U; i < 48U; ++i) {
        got = rc_get(&byte);
        if (got != 1) return (got == 0) ? 2U : 4U;
        rc_version[i] = (char)byte;
        crc = crc_byte(crc, byte);
        if (!byte) {
            got = rc_get(&hi);
            if (got != 1) return (got == 0) ? 2U : 4U;
            got = rc_get(&lo);
            if (got != 1) return (got == 0) ? 2U : 4U;
            rc_crc_calculated = crc;
            rc_crc_received = ((uint16_t)hi << 8U) | lo;
            return (rc_crc_received == crc) ? 1U : 3U;
        }
    }
    return 5U;
}

/* Recogemos bytes de la PC también mientras esperamos a RoboClaw.
 * No iniciamos otra consulta aquí, porque mezclaríamos dos paquetes.
 * STOP/SW2 son paradas por software, no un paro de emergencia físico.
 */
static void pc_collect(void)
{
    uint32_t next, stat;
    if (!pc_enabled) return;
    if (SIUL2.GPDI51.B.PDI_N) stop_pending = 1U;
    stat = LPUART_6.STAT.R;
    if (stat & UART_ERRORS) {
        LPUART_6.STAT.R = UART_ERRORS;
        LPUART_6.FIFO.R = (1UL << 3U) | (1UL << 14U);
        ++pc_rx_faults; stop_pending = 1U;
        pc_tail = pc_head;
    }
    while (LPUART_6.STAT.R & RX_FULL) {
        char byte = (char)LPUART_6.DATA.R;
        next = (pc_head + 1U) % sizeof(pc_ring);
        if (next == pc_tail) {
            ++pc_rx_faults; stop_pending = 1U; pc_tail = pc_head;
        } else { pc_ring[pc_head] = byte; pc_head = next; }
    }
}

static uint32_t write_packet(uint8_t command, const uint8_t *data, uint32_t length)
{
    uint32_t i;
    uint8_t ack;
    int got;
    uint16_t crc = crc_byte(crc_byte(0U, RC_ADDRESS), command);
    /* En las etapas de lectura bloqueamos cualquier escritura al driver. */
    if (LAB_STAGE <= LAB_TELEMETRY) return 9U;
    prepare_request(command);
    if (!uart_put(&LPUART_3, RC_ADDRESS) || !uart_put(&LPUART_3, command)) return 6U;
    for (i = 0U; i < length; ++i) {
        crc = crc_byte(crc, data[i]);
        if (!uart_put(&LPUART_3, data[i])) return 6U;
    }
    if (!uart_put(&LPUART_3, (uint8_t)(crc >> 8U)) ||
        !uart_put(&LPUART_3, (uint8_t)crc)) return 6U;
    got = rc_get(&ack);
    if (got != 1) return (got == 0) ? 2U : 4U;
    return (ack == 0xFFU) ? 1U : 8U;
}

static void stop_motor(const char *reason)
{
    const uint8_t zero_duty[2] = {0U, 0U};
    control_armed = 0U; control_mode = 0U; control_target = 0;
    position_target_set = 0U; position_reached = 0U;
    student_position_reset(&position_control);
    student_pid_reset(&speed_pid, (float)rc_speed_m1_cps);
    pid_reference_cps = 0; pid_error_cps = 0; pid_duty_milli_pct = 0;
    pid_integral_milli_pct = 0; pid_duty_raw = 0; pid_dt_us = 0U;
    /* Mandamos duty cero a M1, sin retención de posición.
     * ACK confirma recepción, no que el eje ya esté detenido.
     */
    rc_stop_status = write_packet(32U, zero_duty, 2U);
    stop_retry_time = cycles();
    if (reason) {
        console("DESARMADO "); console(reason);
        console(" acuse_parada="); console_u32(rc_stop_status); console("\r\n");
    }
}

/* La protección acompaña los límites: margen del 20%, mínimo 500 conteos/s.
 * Calculamos en 64 bits para admitir todo el rango int32 sin desbordamiento.
 */
static uint32_t speed_excessive(void)
{
    int64_t low = speed_min_cps, high = speed_max_cps;
    int64_t extent = -low > high ? -low : high;
    int64_t margin = extent / 5;
    if (margin < 500) margin = 500;
    return (int64_t)rc_speed_m1_cps < low - margin ||
           (int64_t)rc_speed_m1_cps > high + margin;
}

static int32_t bounded_float_i32(float value)
{
    /* INT32_MAX se redondea a 2^31 en float; no convertir ese valor directo. */
    if (value >= 2147483648.0f) return INT32_MAX;
    if (value <= -2147483648.0f) return INT32_MIN;
    return (int32_t)value;
}

static int32_t bounded_error_i32(int32_t reference, int32_t measured)
{
    int64_t error = (int64_t)reference - measured;
    if (error > INT32_MAX) return INT32_MAX;
    if (error < INT32_MIN) return INT32_MIN;
    return (int32_t)error;
}

static uint32_t send_pid_duty(void)
{
    uint8_t payload[2];
    uint16_t encoded;
    pid_duty_milli_pct = (int32_t)(speed_pid.output * 1000.0f);
    pid_integral_milli_pct = (int32_t)(speed_pid.integral * 1000.0f);
    /* DutyM1 (32): int16 con signo, ±32767 = ±100%, byte alto primero.
     * En este modo mandamos duty, no velocidad ni ganancias al RoboClaw.
     */
    pid_duty_raw = (int32_t)(speed_pid.output * (32767.0f / 100.0f));
    encoded = (uint16_t)pid_duty_raw;
    payload[0] = (uint8_t)(encoded >> 8U); payload[1] = (uint8_t)encoded;
    rc_write_status = write_packet(32U, payload, 2U);
    if (rc_write_status != 1U) {
        delay_ms(12U); stop_motor("sin acuse del duty PID"); return 0U;
    }
    return 1U;
}

static void run_speed_pid(void)
{
    uint32_t now = cycles(), elapsed = now - pid_time;
    float dt;
    if (!control_armed || (control_mode != 1U && control_mode != 3U)) return;
    if (!rc_sample_valid || stop_pending ||
        elapsed >= ticks_per_ms * PID_MAX_INTERVAL_MS) {
        stop_motor("datos vencidos, intervalo PID o solicitud de parada"); return;
    }
    if (speed_excessive()) {
        stop_motor("velocidad excesiva en PID"); return;
    }
    if (!elapsed) return;
    dt = (float)elapsed / (float)core_clock_hz;
    pid_time = now;
    pid_dt_us = (uint32_t)(dt * 1000000.0f);
    if (control_mode == 3U) {
        unsigned int was_reached = position_control.reached;
        position_error = bounded_error_i32(position_target, measured_position());
        control_target = bounded_float_i32(student_position_step(&position_control,
            position_target, measured_position(), (float)rc_speed_m1_cps,
            (float)speed_min_cps, (float)speed_max_cps));
        position_reached = position_control.reached;
        if (was_reached && !position_reached)
            student_pid_reset(&speed_pid, (float)rc_speed_m1_cps);
        if (position_reached) {
            student_pid_reset(&speed_pid, (float)rc_speed_m1_cps);
            pid_reference_cps = 0; pid_error_cps = 0;
            (void)send_pid_duty(); return;
        }
    }
    (void)student_pid_step(&speed_pid, (float)control_target,
                          (float)rc_speed_m1_cps, dt);
    pid_reference_cps = bounded_float_i32(speed_pid.reference);
    pid_error_cps = bounded_float_i32(speed_pid.reference - (float)rc_speed_m1_cps);
    ++pid_steps;
    (void)send_pid_duty();
}

static int parse_number(char **cursor, int32_t *value)
{
    char *end;
    long parsed;
    while (**cursor == ' ') ++*cursor;
    if (!**cursor) return 0;
    errno = 0;
    parsed = strtol(*cursor, &end, 10);
    if (end == *cursor || errno == ERANGE || (*end && *end != ' ')) return 0;
    *cursor = end; *value = (int32_t)parsed;
    return 1;
}

static int end_of_line(char *cursor)
{
    while (*cursor == ' ') ++cursor;
    return *cursor == 0;
}

/* INFO sirve para que el panel lea la configuración real, no una suposición. */
static void print_config(void)
{
    console("CONFIG protocolo=2 etapa="); console_u32(LAB_STAGE);
    console(" minimo="); console_i32(speed_min_cps);
    console(" maximo="); console_i32(speed_max_cps);
    console(" kp_milesimas="); console_i32(pid_kp_milli);
    console(" ki_milesimas="); console_i32(pid_ki_milli);
    console(" kd_milesimas="); console_i32(pid_kd_milli);
    console(" duty_maximo="); console_i32(pid_duty_max_pct);
    if (LAB_STAGE == LAB_NXP_POSITION) {
        console(" cero_definido="); console_u32(position_zero_set);
        console(" origen="); console_i32(position_origin);
        console(" ventana="); console_u32(position_window_set);
        console(" pos_min="); console_i32(position_min);
        console(" pos_max="); console_i32(position_max);
        console(" pos_kp_milesimas="); console_i32(position_kp_milli);
        console(" pos_kd_milesimas="); console_i32(position_kd_milli);
        console(" tolerancia="); console_i32(position_tolerance);
    }
    console(" armado="); console_u32(control_armed); console("\r\n");
}

static uint32_t internal_velocity(int32_t value)
{
    uint32_t i, fields[2] = {500U, (uint32_t)value};
    uint8_t payload[8];
    for (i = 0U; i < 8U; ++i)
        payload[i] = (uint8_t)(fields[i / 4U] >> (24U - 8U * (i % 4U)));
    rc_write_status = write_packet(38U, payload, 8U);
    if (rc_write_status != 1U) {
        delay_ms(12U); stop_motor("sin acuse de velocidad interna"); return 0U;
    }
    control_mode = 2U; control_target = value; pid_reference_cps = value;
    return 1U;
}

static void command_line(char *line)
{
    char *args = line;
    int32_t value, maximum, derivative;
    while (*args && *args != ' ') ++args;
    if (*args) *args++ = 0;
    if (!strcmp(line, "INFO") && end_of_line(args)) { print_config(); return; }
    if (!strcmp(line, "HELP") && end_of_line(args)) {
        if (LAB_STAGE <= LAB_TELEMETRY) {
            console("SOLO LECTURA: INFO/HELP. No se mandan ordenes al motor, ni STOP.\r\n");
        } else {
            console("INFO | HELP | ARM | STOP | VEL n | LIMITES min max | WINDOW min max\r\n");
            if (LAB_STAGE >= LAB_NXP_PID)
                console("PID p i d en milesimas | DUTYMAX porcentaje: maximo simetrico 0..100%, inicial 15%.\r\n");
            else console("PID interno RoboClaw; aqui no se mide ni limita su duty.\r\n");
            if (LAB_STAGE == LAB_NXP_POSITION)
                console("ZERO | WINDOW min max | POS conteos | PPID p d en milesimas | TOL conteos. POS absoluto respecto a ZERO.\r\n");
            else console("VEL persiste hasta STOP/SW2/fallo. POS deshabilitado. Rango definido con LIMITES, enteros int32.\r\n");
        }
        return;
    }
    if (LAB_STAGE <= LAB_TELEMETRY) {
        console("RECHAZADO: firmware de lectura, sin escrituras a RoboClaw\r\n"); return;
    }
    if (LAB_STAGE == LAB_INTERNAL_SPEED && !strcmp(line, "PID")) {
        console("RECHAZADO PID: cargar etapa 4; las ganancias internas se ajustan en Motion Studio\r\n"); return;
    }
    if (!strcmp(line, "DUTYMAX") && parse_number(&args, &value) && end_of_line(args)) {
        if (LAB_STAGE < LAB_NXP_PID) {
            console("RECHAZADO DUTYMAX: solo disponible para el PID NXP\r\n"); return;
        }
        if (value < 0 || value > 100 || rc_stop_status != 1U) {
            console("RECHAZADO DUTYMAX: porcentaje entero 0..100 y acuse de parada valido\r\n"); return;
        }
        if (value && control_armed && (stop_pending || !rc_sample_valid || expired(sample_time, 500U))) {
            console("RECHAZADO DUTYMAX: parada pendiente o falta una medicion reciente\r\n"); return;
        }
        pid_duty_max_pct = value;
        student_pid_set_limit(&speed_pid, (float)value);
        if (!value) {
            stop_motor("DUTYMAX 0");
            if (rc_stop_status != 1U) { print_config(); return; }
        } else if (control_armed && (control_mode == 1U || control_mode == 3U)) {
            /* Reenviamos la salida recortada sin esperar a otro periodo PID. */
            if (!send_pid_duty()) { print_config(); return; }
        }
        console("ACEPTADO DUTYMAX: limite aplicado en NXP\r\n"); print_config(); return;
    }
    if (!strcmp(line, "PID") && parse_number(&args, &value) &&
        parse_number(&args, &maximum) && parse_number(&args, &derivative) && end_of_line(args)) {
        if (rc_stop_status != 1U || value < 0 || value > 1000 ||
            maximum < 0 || maximum > 1000 || derivative < 0 || derivative > 1000) {
            console("RECHAZADO PID: ganancias 0..1000 milesimas y acuse de parada valido\r\n"); return;
        }
        if (control_armed && (!rc_sample_valid || expired(sample_time, 500U))) {
            console("RECHAZADO PID: falta una medicion reciente\r\n"); return;
        }
        pid_kp_milli = value; pid_ki_milli = maximum; pid_kd_milli = derivative;
        student_pid_tune(&speed_pid, (float)value / 1000.0f,
            (float)maximum / 1000.0f, (float)derivative / 1000.0f, (float)rc_speed_m1_cps);
        if (!control_armed) student_pid_reset(&speed_pid, (float)rc_speed_m1_cps);
        console("ACEPTADO PID: ganancias aplicadas en NXP\r\n"); print_config(); return;
    }
    if (!strcmp(line, "LIMITES") && parse_number(&args, &value) &&
        parse_number(&args, &maximum) && end_of_line(args)) {
        if (value > 0 || maximum < 0 || value >= maximum || rc_stop_status != 1U ||
            (LAB_STAGE == LAB_NXP_POSITION && (value >= 0 || maximum <= 0))) {
            console(LAB_STAGE == LAB_NXP_POSITION ?
                "RECHAZADO LIMITES: posicion requiere min < 0 < max y acuse valido\r\n" :
                "RECHAZADO LIMITES: min <= 0 <= max; min < max; enteros int32 y acuse de parada valido\r\n"); return;
        }
        speed_min_cps = value; speed_max_cps = maximum;
        /* Si estrechamos el rango, la consigna actual también debe respetarlo. */
        if (control_mode != 3U && (control_target < value || control_target > maximum)) {
            int32_t bounded = control_target < value ? value : maximum;
            if (!bounded) stop_motor("limites nuevos dejan la consigna en cero");
            else if (LAB_STAGE == LAB_INTERNAL_SPEED) (void)internal_velocity(bounded);
            else control_target = bounded;
        }
        console("ACEPTADO LIMITES\r\n"); print_config(); return;
    }
    if (!strcmp(line, "ZERO") && end_of_line(args) && LAB_STAGE == LAB_NXP_POSITION) {
        if (control_armed || stop_pending || rc_stop_status != 1U || !rc_sample_valid || expired(sample_time, 500U) ||
            rc_speed_m1_cps < -10 || rc_speed_m1_cps > 10) {
            console("RECHAZADO ZERO: desarmar, lectura reciente, acuse valido y motor detenido\r\n"); return;
        }
        position_origin = rc_position_m1; position_zero_set = 1U; position_window_set = 0U;
        position_min = 0; position_max = 0; position_target_set = 0U;
        student_position_reset(&position_control);
        console("ACEPTADO ZERO: cero logico; definir WINDOW antes de armar\r\n"); print_config(); return;
    }
    if (!strcmp(line, "PPID") && LAB_STAGE == LAB_NXP_POSITION &&
        parse_number(&args, &value) && parse_number(&args, &maximum) && end_of_line(args)) {
        if (value < 0 || value > 100000 || maximum < 0 || maximum > 100000 || rc_stop_status != 1U ||
            (control_armed && (!rc_sample_valid || expired(sample_time, 500U)))) {
            console("RECHAZADO PPID: ganancias 0..100000 milesimas y datos/acuse validos\r\n"); return;
        }
        position_kp_milli = value; position_kd_milli = maximum;
        position_control.kp = (float)value / 1000.0f; position_control.kd = (float)maximum / 1000.0f;
        console("ACEPTADO PPID: lazo externo de posicion P/PD\r\n"); print_config(); return;
    }
    if (!strcmp(line, "TOL") && LAB_STAGE == LAB_NXP_POSITION &&
        parse_number(&args, &value) && end_of_line(args)) {
        if (value <= 0 || rc_stop_status != 1U ||
            (control_armed && (!rc_sample_valid || expired(sample_time, 500U)))) {
            console("RECHAZADO TOL: conteos positivos y datos/acuse validos\r\n"); return;
        }
        position_tolerance = value; position_control.tolerance = value;
        student_position_reset(&position_control); position_reached = 0U;
        console("ACEPTADO TOL\r\n"); print_config(); return;
    }
    if (!strcmp(line, "POS") && LAB_STAGE == LAB_NXP_POSITION &&
        parse_number(&args, &value) && end_of_line(args)) {
        if (!control_armed || stop_pending || !rc_sample_valid || expired(sample_time, 500U) ||
            !position_zero_set || !position_window_set || value <= position_min || value >= position_max) {
            console("RECHAZADO POS: armar, datos recientes y destino estrictamente dentro de WINDOW\r\n"); return;
        }
        if (control_mode != 3U) {
            student_pid_reset(&speed_pid, (float)rc_speed_m1_cps); pid_time = cycles();
        }
        position_target = value; position_target_set = 1U;
        position_error = bounded_error_i32(value, measured_position());
        student_position_reset(&position_control); position_reached = 0U;
        control_mode = 3U;
        console("ACEPTADO POS NXP "); console_i32(value); console("\r\n"); return;
    }
    if (!strcmp(line, "POS")) {
        console(LAB_STAGE == LAB_NXP_POSITION ? "RECHAZADO POS: usar POS conteos, entero int32\r\n" :
            "RECHAZADO POS: cargar etapa de posicion\r\n"); return;
    }
    if (!strcmp(line, "STOP") && end_of_line(args)) {
        stop_motor("STOP desde PC"); return;
    }
    if (!strcmp(line, "WINDOW") && parse_number(&args, &value) &&
        parse_number(&args, &maximum) && end_of_line(args)) {
        if (control_armed || stop_pending || rc_stop_status != 1U || value >= maximum ||
            (LAB_STAGE == LAB_NXP_POSITION && (!position_zero_set || !rc_sample_valid ||
             expired(sample_time, 500U) || measured_position() <= value || measured_position() >= maximum))) {
            console(LAB_STAGE == LAB_NXP_POSITION ?
                "RECHAZADO WINDOW: ZERO, desarmar, lectura/acuse validos y posicion dentro de min/max\r\n" :
                "RECHAZADO WINDOW: desarmar, acuse valido y min < max\r\n"); return;
        }
        position_min = value; position_max = maximum; position_window_set = 1U;
        console("ACEPTADO WINDOW: comprobar antes el recorrido fisico\r\n"); print_config(); return;
    }
    if (!strcmp(line, "ARM") && end_of_line(args)) {
        if ((LAB_STAGE >= LAB_NXP_PID && pid_duty_max_pct == 0) ||
            (LAB_STAGE == LAB_NXP_POSITION && (!position_zero_set || !position_window_set ||
             speed_min_cps >= 0 || speed_max_cps <= 0)) ||
            pid_selftest_errors || stop_pending || SIUL2.GPDI51.B.PDI_N || rc_stop_status != 1U ||
            !rc_sample_valid || expired(sample_time, 500U) ||
            rc_speed_m1_cps < -10 || rc_speed_m1_cps > 10) {
            console("RECHAZADO ARM: revisar duty maximo, autoprueba, SW2, datos/acuse validos y motor detenido\r\n");
            return;
        }
        if (position_window_set &&
            (measured_position() <= position_min || measured_position() >= position_max)) {
            console("RECHAZADO ARM: posicion fuera de WINDOW\r\n"); return;
        }
        control_armed = 1U;
        console("ARMADO M1: enviar "); console(LAB_STAGE == LAB_NXP_POSITION ? "POS" : "VEL");
        console("; STOP o SW2 para detener\r\n"); print_config(); return;
    }
    if (!strcmp(line, "VEL") &&
        parse_number(&args, &value) && end_of_line(args)) {
        if (!strcmp(line, "VEL") && value == 0) { stop_motor("VEL 0"); return; }
        if (LAB_STAGE == LAB_NXP_POSITION) {
            console("RECHAZADO VEL: etapa de posicion; usar POS\r\n"); return;
        }
        if (!control_armed || stop_pending || !rc_sample_valid || expired(sample_time, 500U)) {
            console("RECHAZADO movimiento: armar y comprobar lectura reciente\r\n"); return;
        }
        if (value < speed_min_cps || value > speed_max_cps) {
            console("RECHAZADO VEL: fuera de LIMITES; sigue activa la consigna anterior\r\n"); return;
        }
        if (LAB_STAGE == LAB_INTERNAL_SPEED) {
            if (internal_velocity(value)) {
                console("ACEPTADO VEL ROBOCLAW "); console_i32(value); console("\r\n");
            }
            return;
        }
        if (control_mode != 1U) {
            student_pid_reset(&speed_pid, (float)rc_speed_m1_cps);
            pid_time = cycles();
        }
        control_mode = 1U;
        control_target = value;
        console("ACEPTADO VEL PID NXP "); console_i32(value); console("\r\n");
        return;
    }
    console("RECHAZADO formato: comando en mayusculas, enteros y Enter. Enviar HELP\r\n");
}

static void service_commands(void)
{
    static char line[64];
    static uint32_t count, discard, previous_faults;
    pc_collect();
    if (previous_faults != pc_rx_faults) {
        previous_faults = pc_rx_faults; count = 0U; discard = 1U;
    }
    if (LAB_STAGE <= LAB_TELEMETRY) stop_pending = 0U;
    if (stop_pending) {
        stop_pending = 0U; pc_tail = pc_head; count = 0U; discard = 0U;
        stop_motor("SW2 o fallo de recepcion PC"); return;
    }
    /* No hay vencimiento por inactividad de PC. Cerrar el monitor no para M1. */
    while (pc_tail != pc_head) {
        char byte = pc_ring[pc_tail];
        pc_tail = (pc_tail + 1U) % sizeof(pc_ring);
        if (byte == '\r' || byte == '\n') {
            if (!discard && count) { line[count] = 0; command_line(line); }
            count = 0U; discard = 0U;
        } else if (!discard) {
            if (byte < 32 || byte > 126 || count >= sizeof(line) - 1U) {
                discard = 1U; count = 0U;
                if (LAB_STAGE > LAB_TELEMETRY) stop_motor("linea PC invalida o demasiado larga");
            } else line[count++] = byte;
        }
    }
    if (control_armed && position_window_set && rc_position_status == 1U &&
        (measured_position() <= position_min || measured_position() >= position_max))
        stop_motor("limite de WINDOW alcanzado");
    if (LAB_STAGE > LAB_TELEMETRY && rc_stop_status != 1U && expired(stop_retry_time, 200U)) {
        delay_ms(12U); stop_motor(0); /* Solo reintentamos duty cero. */
    }
}

/* Si estamos desarmados, aquí podemos pausar sin cortar un paquete UART. */
__attribute__((noinline)) void rc_sample_ready(void)
{
    __asm volatile ("nop");
}

static void handle_telemetry_fault(void)
{
    if (!rc_sample_valid && control_armed) {
        delay_ms(12U); stop_motor("fallo de lectura");
    }
    print_speed_fault();
}

int main(void)
{
    uint32_t result, i;
    pid_selftest_errors = student_pid_selftest() | (student_position_selftest() << 16U);
    led_init();
    result = clocks_init();
    if (result) { rc_status = result; led_result(0U); for (;;) { } }
    /* TX: PTD2 MSCR98 SSS6; RX: PTD3 MSCR99, IMCR702-512=190 SSS3. */
    SIUL2.MSCR[98].R = (1UL << 21U) | 6U;
    SIUL2.MSCR[99].R = 1UL << 19U;
    SIUL2.IMCR[190].R = 3U;
    /* Consola USB OpenSDA: PTA16 TX SSS5; PTA15 RX, IMCR705-512=193 SSS2. */
    SIUL2.MSCR[16].R = (1UL << 21U) | 5U;
    SIUL2.MSCR[15].R = 1UL << 19U;
    SIUL2.IMCR[193].R = 2U;
    result = uart_init(&LPUART_3, 38400U);
    if (!result) result = uart_init(&LPUART_6, 115200U);
    if (result) { rc_status = result; led_result(0U); for (;;) { } }
    /* La cola de UART6 recibe ráfagas mientras atendemos UART3. */
    LPUART_6.CTRL.R = 0U;
    LPUART_6.FIFO.R = (1UL << 3U) | (1UL << 14U) | (1UL << 15U);
    LPUART_6.CTRL.R = (1UL << 18U) | (1UL << 19U);
    pc_enabled = 1U;
    delay_ms(500U);
    console("\r\nS32K312: direccion=128, baud=38400, etapa="); console_u32(LAB_STAGE);
    console("\r\nPERFIL ");
    if (LAB_STAGE == LAB_VERSION) console("VERSION\r\n");
    else if (LAB_STAGE == LAB_TELEMETRY) console("LECTURA\r\n");
    else if (LAB_STAGE == LAB_INTERNAL_SPEED) console("VELOCIDAD ROBOCLAW\r\n");
    else if (LAB_STAGE == LAB_NXP_PID) console("PID NXP\r\n");
    else console("POSICION NXP\r\n");
    rc_version_status = read_version();
    if (rc_version_status == 1U) {
        console("OK version: ");
        for (i = 0U; i < 64U && rc_version[i]; ++i)
            if (!uart_put(&LPUART_6, (uint8_t)rc_version[i])) break;
        console("\r\n");
    } else {
        console("Error de version, estado="); console_u32(rc_version_status); console("\r\n");
    }
    if (LAB_STAGE > LAB_TELEMETRY) stop_motor("arranque");
    console("Errores de autoprueba PID="); console_u32(pid_selftest_errors); console(" (0=correcto)\r\n");
    print_config();
    console("Enviar HELP para ver comandos.\r\n");
    if (LAB_STAGE > LAB_TELEMETRY)
        console("AVISO: sin vencimiento PC. Cerrar el monitor NO para el motor; usar STOP o SW2.\r\n");
    else console("SOLO LECTURA: no mandamos duty cero. Detener el motor ANTES de cargar este programa.\r\n");
    poll_time = cycles() - ticks_per_ms * PID_PERIOD_MS;
    print_time = cycles();
    for (;;) {
        service_commands();
        if (LAB_STAGE == LAB_VERSION) {
            if (expired(print_time, 1000U)) {
                print_time = cycles(); ++rc_attempts;
                rc_version_status = read_version(); rc_status = rc_version_status;
                if (rc_status == 1U) {
                    ++rc_successes; console("OK version: ");
                    for (i = 0U; i < 64U && rc_version[i]; ++i)
                        (void)uart_put(&LPUART_6, (uint8_t)rc_version[i]);
                    console("\r\n");
                } else {
                    ++rc_failures; console("Error de version, estado=");
                    console_u32(rc_status); console("\r\n");
                }
                led_result(rc_status == 1U); rc_sample_ready();
            }
            continue;
        }
        if (expired(poll_time, PID_PERIOD_MS)) {
            uint32_t scheduled = poll_time + ticks_per_ms * PID_PERIOD_MS;
            poll_time = cycles();
            if ((uint32_t)(poll_time - scheduled) < ticks_per_ms * PID_PERIOD_MS)
                poll_time = scheduled; /* No hacemos una ráfaga para recuperar retrasos. */
            ++rc_attempts;
            read_telemetry(); sample_time = cycles();
            if (rc_sample_valid) ++rc_successes; else ++rc_failures;
            handle_telemetry_fault();
            service_commands();
            if (control_armed && speed_excessive())
                stop_motor("velocidad excesiva");
            if (LAB_STAGE == LAB_INTERNAL_SPEED && control_armed && control_mode == 2U)
                pid_error_cps = bounded_error_i32(control_target, rc_speed_m1_cps);
            run_speed_pid();
            if (expired(print_time, 200U)) {
                print_time = cycles(); print_telemetry();
            }
            led_result(rc_sample_valid);
            rc_sample_ready();
        }
    }
}
