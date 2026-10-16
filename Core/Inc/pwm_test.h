#ifndef PWM_TEST_H
#define PWM_TEST_H

#include <stdint.h>

/*
 * Caracterizacion de la zona muerta de los motores: busca, por rueda y por
 * sentido de giro, el PWM minimo con el que la rueda
 *  - ARRANCA desde el reposo (friccion estatica, "pwm_start"), y
 *  - SE MANTIENE girando una vez que ya esta en movimiento (friccion
 *    dinamica, "pwm_stop"). Este es el valor relevante cuando el control de
 *    trayectoria va bajando la velocidad al acercarse al objetivo.
 *
 * Convencion de ruedas y signos igual a stm_unified (control_PID.c):
 * indice 0=FR, 1=FL, 2=BR, 3=BL; signo positivo = la rueda gira en el
 * sentido que hace avanzar al robot.
 *
 * El PWM se expresa en cuentas del timer (0..999 -> 0..100 % de duty).
 */

#define PWMTEST_N_WHEELS        4U

/* Tamaños de los mensajes publicados (Float64MultiArray). */
#define PWMTEST_STATUS_SIZE     10U
#define PWMTEST_RESULT_SIZE     10U

typedef enum
{
    PWMTEST_MODE_ABORT  = 0, /* Detiene cualquier prueba en curso. */
    PWMTEST_MODE_SINGLE = 1, /* Una rueda a la vez, las demas en 0. */
    PWMTEST_MODE_GROUP  = 2  /* Las 4 ruedas juntas siguiendo un patron. */
} PwmTest_Mode_t;

typedef enum
{
    PWMTEST_PATTERN_FORWARD  = 0, /* [FR, FL, BR, BL] = [+, +, +, +] */
    PWMTEST_PATTERN_LATERAL  = 1, /* [+, -, -, +] -> desplazamiento a la izquierda */
    PWMTEST_PATTERN_ROTATION = 2  /* [+, -, +, -] -> giro antihorario */
} PwmTest_Pattern_t;

typedef enum
{
    PWMTEST_ST_IDLE      = 0,
    PWMTEST_ST_REST      = 1, /* PWM en 0 esperando que la rueda quede quieta. */
    PWMTEST_ST_RAMP_UP   = 2, /* Subiendo PWM hasta detectar arranque. */
    PWMTEST_ST_BOOST     = 3, /* Empujon corto para asegurar regimen dinamico. */
    PWMTEST_ST_RAMP_DOWN = 4, /* Bajando PWM hasta detectar que se detiene. */
    PWMTEST_ST_DONE      = 5,
    PWMTEST_ST_ABORTED   = 6,
    PWMTEST_ST_REJECTED  = 7  /* Comando con parametros invalidos. */
} PwmTest_State_t;

/* Bits del campo flags de cada resultado. */
#define PWMTEST_FLAG_START_OK   0x01U /* pwm_start valido. */
#define PWMTEST_FLAG_STOP_OK    0x02U /* pwm_stop valido. */
#define PWMTEST_FLAG_WRONG_DIR  0x04U /* Se vio giro en sentido contrario al pedido. */

typedef struct
{
    int32_t mode;      /* PwmTest_Mode_t */
    int32_t target;    /* SINGLE: mascara de ruedas (bit0=FR .. bit3=BL); GROUP: PwmTest_Pattern_t */
    int32_t dir;       /* +1, -1 o 0 (= ambos sentidos) */
    int32_t trials;    /* Repeticiones finas por combinacion (ademas de la gruesa). */
    int32_t pwm_limit; /* PWM maximo permitido durante la prueba (cuentas). */
} PwmTest_Cmd_t;

typedef enum
{
    PWMTEST_EVT_STATUS = 0,
    PWMTEST_EVT_RESULT = 1
} PwmTest_EventType_t;

/*
 * Evento generado por la tarea de prueba para que la tarea micro-ROS lo
 * publique.
 *
 * STATUS -> /pwm_test/status:
 *   [state, mode, target, dir, trial, pwm, vel_fr, vel_fl, vel_br, vel_bl]
 *   target = rueda (SINGLE) o patron (GROUP); vel en rad/s promediadas en la
 *   ventana de medicion del escalon.
 *
 * RESULT -> /pwm_test/result (uno por rueda y por intento):
 *   [mode, pattern, wheel, dir, trial, pwm_start, pwm_stop,
 *    vel_start, vel_stop, flags]
 *   pattern = -1 en modo SINGLE; trial 0 = barrido grueso.
 */
typedef struct
{
    uint8_t type; /* PwmTest_EventType_t */
    uint8_t size;
    float data[PWMTEST_RESULT_SIZE];
} PwmTest_Event_t;

/* Deja PWM en cero, encoders referenciados y crea la cola de eventos. */
void PwmTest_Init(void);

/* Ejecuta un paso de 10 ms de la maquina de estados de la prueba. */
void PwmTest_Update(void);

/* Encola un comando nuevo (se procesa en el siguiente PwmTest_Update). */
void PwmTest_Command(const PwmTest_Cmd_t *cmd);

/* Saca un evento pendiente; devuelve 1 si habia alguno. No bloquea. */
uint8_t PwmTest_PopEvent(PwmTest_Event_t *evt);

#endif
