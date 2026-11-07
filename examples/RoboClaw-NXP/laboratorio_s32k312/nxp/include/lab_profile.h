/* Aquí elegimos qué práctica cargar. Por defecto solo leemos datos.
 * Cada archivo de ejemplos fija su propia etapa antes de incluir este header.
 */
#ifndef LAB_PROFILE_H
#define LAB_PROFILE_H
#define LAB_VERSION 1
#define LAB_TELEMETRY 2
#define LAB_INTERNAL_SPEED 3
#define LAB_NXP_PID 4
#define LAB_NXP_POSITION 5
#ifndef LAB_STAGE
#define LAB_STAGE LAB_TELEMETRY
#endif
#if LAB_STAGE < LAB_VERSION || LAB_STAGE > LAB_NXP_POSITION
#error "LAB_STAGE: 1 version, 2 lectura, 3 velocidad RoboClaw, 4 PID NXP, 5 posicion NXP"
#endif
#endif
