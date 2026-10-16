#include "pwm_test.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "tim.h"

/*
 * Prueba de PWM minimo de los motores.
 *
 * Metodo (por rueda, por sentido de giro):
 *
 *  1. REST: PWM en 0 hasta que la rueda este quieta.
 *  2. RAMP_UP: se sube el PWM en escalones. Cada escalon dura STEP_MS; los
 *     primeros SETTLE_MS se descartan (transitorio) y en los WINDOW_MS
 *     restantes se cuentan pulsos de encoder. Se considera que la rueda se
 *     mueve cuando en la ventana hay al menos MOVE_COUNTS pulsos en el
 *     sentido pedido, durante CONFIRM_STEPS escalones seguidos. El PWM del
 *     primero de esos escalones es pwm_start (friccion estatica).
 *  3. BOOST: empujon corto por encima de pwm_start para que la rueda quede
 *     claramente en regimen dinamico.
 *  4. RAMP_DOWN: se baja el PWM en escalones hasta que la rueda deja de
 *     moverse CONFIRM_STEPS escalones seguidos. El ultimo PWM con movimiento
 *     es pwm_stop: el minimo que SOSTIENE el giro (friccion dinamica), que es
 *     el que importa cuando el control de trayectoria reduce la velocidad al
 *     acercarse al objetivo.
 *
 * Se cuentan pulsos acumulados en una ventana larga (200 ms) en vez de usar
 * la velocidad instantanea a 10 ms: con 384.5 pulsos/vuelta, un pulso en
 * 10 ms equivale a 1.63 rad/s, asi que a baja velocidad la medicion
 * instantanea es casi todo ruido de cuantizacion. En 200 ms la resolucion
 * baja a ~0.08 rad/s.
 *
 * Cada combinacion (rueda o patron, sentido) empieza con un barrido grueso
 * (trial 0, escalon COARSE_STEP desde 0) que ubica aproximadamente los
 * umbrales, y luego `trials` barridos finos (escalon FINE_STEP) que empiezan
 * FINE_MARGIN cuentas por debajo de lo encontrado. Asi la prueba no tarda
 * minutos por rueda y el robot se desplaza poco en el modo GROUP.
 */

/* ===== Parametros fisicos (mismos que stm_unified/control_PID.c) ===== */
#define PPR                 384.5f
#define TWO_PI_F            6.2831853f
#define PWM_MAX             999

/* ===== Temporizacion ===== */
#define TASK_PERIOD_MS      10U
#define STEP_MS             300U
#define SETTLE_MS           100U
#define WINDOW_MS           (STEP_MS - SETTLE_MS)
#define REST_MS             1000U
#define BOOST_MS            500U
#define IDLE_STATUS_MS      1000U

/* ===== Deteccion de movimiento ===== */
#define MOVE_COUNTS         4    /* ~0.33 rad/s en una ventana de 200 ms. */
#define CONFIRM_STEPS       2

/* ===== Barridos ===== */
#define COARSE_STEP         10
#define FINE_STEP           2
#define FINE_MARGIN         40
#define BOOST_PWM           40

/* ===== Limites de comando ===== */
#define DEFAULT_TRIALS      3
#define MAX_TRIALS          10
#define DEFAULT_PWM_LIMIT   600
#define MIN_PWM_LIMIT       50

#define EVT_QUEUE_LEN       32U

enum { W_FR = 0, W_FL = 1, W_BR = 2, W_BL = 3 };

/*
 * Signos de cada rueda [FR, FL, BR, BL] para los patrones del modo GROUP,
 * deducidos de la cinematica de odometry_omni.c:
 *   vx ∝  fl + fr + rl + rr
 *   vy ∝ -fl + fr + rl - rr
 *   wz ∝ -fl + fr - rl + rr
 */
static const int8_t k_patterns[3][PWMTEST_N_WHEELS] = {
    { +1, +1, +1, +1 }, /* FORWARD  */
    { +1, -1, -1, +1 }, /* LATERAL  */
    { +1, -1, +1, -1 }  /* ROTATION */
};

/*
 * Asignacion fisica (ver README de stm_unified):
 * - htim1: encoder FR   - htim2 CH1/CH4: PWM FR
 * - htim3: encoder FL   - htim2 CH2/CH3: PWM FL
 * - htim5: encoder BR   - htim12 CH1/CH2: PWM BR
 * - htim8: encoder BL   - htim10 CH1 (Adelante) / htim13 CH1 (Atras): PWM BL
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

typedef struct
{
    int8_t  sign;          /* Sentido esperado en el trial actual; 0 = inactiva. */
    int32_t last_cnt;      /* Ultima lectura cruda del contador. */
    int32_t acc;           /* Pulsos acumulados (convencion fisica). */
    int32_t base;          /* acc al inicio de la ventana de medicion. */
    float   vel;           /* Velocidad de la ultima ventana, rad/s. */

    uint8_t started;
    uint8_t stopped;
    uint8_t moved_in_descent;
    uint8_t flags;
    int32_t consec;        /* Escalones consecutivos cumpliendo la condicion. */
    int32_t cand_pwm;      /* PWM del primer escalon de la racha actual. */
    float   cand_vel;
    int32_t start_pwm;
    float   start_vel;
    int32_t stop_pwm;
    float   stop_vel;
} Wheel_t;

typedef struct
{
    PwmTest_State_t state;

    /* Comando en ejecucion. */
    int32_t mode;
    int32_t mask;
    int32_t pattern;
    int32_t trials;
    int32_t pwm_limit;
    int8_t  dirs[2];
    int32_t n_dirs;

    /* Posicion dentro de la secuencia de trabajos. */
    int32_t wheel;         /* Rueda actual (solo SINGLE). */
    int32_t dir_idx;
    int32_t trial;         /* 0 = barrido grueso. */

    /* Referencia obtenida en el barrido grueso de la combinacion actual. */
    uint8_t ref_valid;
    int32_t ref_start_min;
    int32_t ref_stop_max;

    /* Estado del trial en curso. */
    int32_t pwm;
    int32_t step;
    uint32_t elapsed_ms;

    uint32_t idle_ms;
} Test_t;

static Wheel_t w[PWMTEST_N_WHEELS];
static Test_t t;

static QueueHandle_t evt_queue = NULL;

static volatile uint8_t cmd_pending = 0U;
static PwmTest_Cmd_t cmd_buffer;

/* ========================================================================= */
/* Hardware                                                                  */
/* ========================================================================= */

static int16_t read_encoder_raw(uint32_t i)
{
    switch (i)
    {
        case W_FR: return (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        case W_FL: return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        case W_BR: return (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
        default:   return (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    }
}

/*
 * Aplica un PWM con signo en convencion fisica (positivo = avance).
 * Misma convencion de montaje que stm_unified: las ruedas izquierdas (FL, BL)
 * invierten el comando; las derechas (FR, BR) invierten el encoder.
 */
static void set_motor(uint32_t i, int32_t pwm_signed)
{
    uint32_t fwd;
    uint32_t rev;

    if (i == W_FL || i == W_BL)
    {
        pwm_signed = -pwm_signed;
    }

    if (pwm_signed > PWM_MAX) pwm_signed = PWM_MAX;
    if (pwm_signed < -PWM_MAX) pwm_signed = -PWM_MAX;

    fwd = (pwm_signed > 0) ? (uint32_t)pwm_signed : 0U;
    rev = (pwm_signed < 0) ? (uint32_t)(-pwm_signed) : 0U;

    switch (i)
    {
        case W_FR: TIM2->CCR1 = fwd;  TIM2->CCR4 = rev;  break;
        case W_FL: TIM2->CCR2 = fwd;  TIM2->CCR3 = rev;  break;
        case W_BR: TIM12->CCR1 = fwd; TIM12->CCR2 = rev; break;
        default:   TIM10->CCR1 = fwd; TIM13->CCR1 = rev; break;
    }
}

static void stop_all_motors(void)
{
    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        set_motor(i, 0);
    }
}

static void apply_pwm(int32_t pwm)
{
    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        set_motor(i, (int32_t)w[i].sign * pwm);
    }
}

static void update_encoders(void)
{
    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        int16_t cnt = read_encoder_raw(i);
        /* La resta en 16 bits resuelve el wrap-around del contador. */
        int16_t delta = (int16_t)(cnt - (int16_t)w[i].last_cnt);
        w[i].last_cnt = cnt;

        if (i == W_FR || i == W_BR)
        {
            delta = (int16_t)(-delta);
        }
        w[i].acc += delta;
    }
}

static void set_led(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ========================================================================= */
/* Eventos                                                                   */
/* ========================================================================= */

static void push_event(const PwmTest_Event_t *evt)
{
    if (evt_queue != NULL)
    {
        /* Si la cola esta llena se descarta: nunca se bloquea el lazo. */
        (void)xQueueSend(evt_queue, evt, 0);
    }
}

static int8_t current_dir(void)
{
    return t.dirs[t.dir_idx];
}

static void emit_status(void)
{
    PwmTest_Event_t evt;

    evt.type = PWMTEST_EVT_STATUS;
    evt.size = PWMTEST_STATUS_SIZE;
    evt.data[0] = (float)t.state;
    evt.data[1] = (float)t.mode;
    evt.data[2] = (float)((t.mode == PWMTEST_MODE_GROUP) ? t.pattern : t.wheel);
    evt.data[3] = (float)((t.n_dirs > 0) ? current_dir() : 0);
    evt.data[4] = (float)t.trial;
    evt.data[5] = (float)t.pwm;
    evt.data[6] = w[W_FR].vel;
    evt.data[7] = w[W_FL].vel;
    evt.data[8] = w[W_BR].vel;
    evt.data[9] = w[W_BL].vel;
    push_event(&evt);
}

static void emit_result(uint32_t i)
{
    PwmTest_Event_t evt;

    evt.type = PWMTEST_EVT_RESULT;
    evt.size = PWMTEST_RESULT_SIZE;
    evt.data[0] = (float)t.mode;
    evt.data[1] = (float)((t.mode == PWMTEST_MODE_GROUP) ? t.pattern : -1);
    evt.data[2] = (float)i;
    evt.data[3] = (float)current_dir();
    evt.data[4] = (float)t.trial;
    evt.data[5] = (w[i].flags & PWMTEST_FLAG_START_OK) ? (float)w[i].start_pwm : -1.0f;
    evt.data[6] = (w[i].flags & PWMTEST_FLAG_STOP_OK) ? (float)w[i].stop_pwm : -1.0f;
    evt.data[7] = w[i].start_vel;
    evt.data[8] = w[i].stop_vel;
    evt.data[9] = (float)w[i].flags;
    push_event(&evt);
}

/* ========================================================================= */
/* Secuencia de trabajos                                                     */
/* ========================================================================= */

static void enter_state(PwmTest_State_t st)
{
    t.state = st;
    t.elapsed_ms = 0U;
    emit_status();
}

static void finish_all(PwmTest_State_t final_state)
{
    stop_all_motors();
    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        w[i].sign = 0;
    }
    t.pwm = 0;
    set_led(0U);
    enter_state(final_state);
}

/* Prepara las ruedas activas y arranca un trial (siempre desde REST). */
static void begin_trial(void)
{
    int8_t dir = current_dir();

    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        if (t.mode == PWMTEST_MODE_GROUP)
        {
            w[i].sign = (int8_t)(k_patterns[t.pattern][i] * dir);
        }
        else
        {
            w[i].sign = ((int32_t)i == t.wheel) ? dir : 0;
        }

        w[i].started = 0U;
        w[i].stopped = 0U;
        w[i].moved_in_descent = 0U;
        w[i].flags = 0U;
        w[i].consec = 0;
        w[i].start_pwm = -1;
        w[i].stop_pwm = -1;
        w[i].start_vel = 0.0f;
        w[i].stop_vel = 0.0f;
        w[i].vel = 0.0f;
    }

    t.pwm = 0;
    stop_all_motors();
    enter_state(PWMTEST_ST_REST);
}

/* Busca la siguiente rueda de la mascara a partir de `from`; -1 si no hay. */
static int32_t next_wheel_in_mask(int32_t from)
{
    for (int32_t i = from; i < (int32_t)PWMTEST_N_WHEELS; i++)
    {
        if ((t.mask & (1 << i)) != 0)
        {
            return i;
        }
    }
    return -1;
}

/*
 * Avanza a la siguiente combinacion (trial -> sentido -> rueda). `skip_fine`
 * se usa cuando el barrido grueso fallo: repetir los finos no tiene sentido.
 */
static void advance_job(uint8_t skip_fine)
{
    t.trial++;
    if (skip_fine || t.trial > t.trials)
    {
        t.trial = 0;
        t.ref_valid = 0U;
        t.dir_idx++;

        if (t.dir_idx >= t.n_dirs)
        {
            t.dir_idx = 0;

            if (t.mode == PWMTEST_MODE_GROUP)
            {
                finish_all(PWMTEST_ST_DONE);
                return;
            }

            t.wheel = next_wheel_in_mask(t.wheel + 1);
            if (t.wheel < 0)
            {
                finish_all(PWMTEST_ST_DONE);
                return;
            }
        }
    }

    begin_trial();
}

/* Cierra el trial actual: publica resultados y actualiza la referencia gruesa. */
static void end_trial(void)
{
    uint8_t any_start = 0U;
    int32_t start_min = PWM_MAX;
    int32_t stop_max = -1;

    stop_all_motors();

    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        if (w[i].sign == 0)
        {
            continue;
        }

        emit_result(i);

        if (w[i].flags & PWMTEST_FLAG_START_OK)
        {
            any_start = 1U;
            if (w[i].start_pwm < start_min) start_min = w[i].start_pwm;
        }
        if ((w[i].flags & PWMTEST_FLAG_STOP_OK) && w[i].stop_pwm > stop_max)
        {
            stop_max = w[i].stop_pwm;
        }
    }

    if (t.trial == 0)
    {
        t.ref_valid = any_start;
        t.ref_start_min = start_min;
        t.ref_stop_max = stop_max;
    }

    advance_job((t.trial == 0 && !any_start) ? 1U : 0U);
}

/* ========================================================================= */
/* Fases                                                                     */
/* ========================================================================= */

static float window_velocity(int32_t counts)
{
    return (TWO_PI_F * (float)counts) / (PPR * ((float)WINDOW_MS / 1000.0f));
}

/*
 * Lleva el reloj de un escalon: guarda la base de la ventana al terminar el
 * transitorio y devuelve 1 cuando el escalon termino y hay medicion nueva.
 */
static uint8_t step_tick(void)
{
    t.elapsed_ms += TASK_PERIOD_MS;

    if (t.elapsed_ms == SETTLE_MS)
    {
        for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
        {
            w[i].base = w[i].acc;
        }
    }

    if (t.elapsed_ms < STEP_MS)
    {
        return 0U;
    }

    t.elapsed_ms = 0U;
    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        int32_t counts = w[i].acc - w[i].base;
        w[i].vel = window_velocity(counts);

        /* Movimiento en sentido opuesto al pedido: cableado/signo sospechoso. */
        if (w[i].sign != 0 && (counts * w[i].sign) <= -MOVE_COUNTS)
        {
            w[i].flags |= PWMTEST_FLAG_WRONG_DIR;
        }
    }
    return 1U;
}

static uint8_t is_moving(uint32_t i)
{
    int32_t counts = w[i].acc - w[i].base;
    return ((counts * w[i].sign) >= MOVE_COUNTS) ? 1U : 0U;
}

static void phase_rest(void)
{
    t.elapsed_ms += TASK_PERIOD_MS;
    if (t.elapsed_ms < REST_MS)
    {
        return;
    }

    if (t.trial == 0 || !t.ref_valid)
    {
        t.step = COARSE_STEP;
        t.pwm = COARSE_STEP;
    }
    else
    {
        t.step = FINE_STEP;
        t.pwm = t.ref_start_min - FINE_MARGIN;
        if (t.pwm < FINE_STEP) t.pwm = FINE_STEP;
    }

    apply_pwm(t.pwm);
    enter_state(PWMTEST_ST_RAMP_UP);
}

static void phase_ramp_up(void)
{
    uint8_t all_started = 1U;

    if (!step_tick())
    {
        return;
    }

    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        if (w[i].sign == 0 || w[i].started)
        {
            continue;
        }

        if (is_moving(i))
        {
            if (w[i].consec == 0)
            {
                w[i].cand_pwm = t.pwm;
                w[i].cand_vel = w[i].vel;
            }
            w[i].consec++;

            if (w[i].consec >= CONFIRM_STEPS)
            {
                w[i].started = 1U;
                w[i].start_pwm = w[i].cand_pwm;
                w[i].start_vel = w[i].cand_vel;
                w[i].flags |= PWMTEST_FLAG_START_OK;
            }
        }
        else
        {
            w[i].consec = 0;
        }

        if (!w[i].started)
        {
            all_started = 0U;
        }
    }

    emit_status();

    if (all_started)
    {
        int32_t start_max = 0;
        for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
        {
            if (w[i].sign != 0 && w[i].start_pwm > start_max) start_max = w[i].start_pwm;
        }

        t.pwm = start_max + BOOST_PWM;
        if (t.pwm > t.pwm_limit) t.pwm = t.pwm_limit;
        apply_pwm(t.pwm);
        enter_state(PWMTEST_ST_BOOST);
        return;
    }

    t.pwm += t.step;
    if (t.pwm > t.pwm_limit)
    {
        /* No arranco con el PWM maximo permitido: se reporta como fallo. */
        end_trial();
        return;
    }
    apply_pwm(t.pwm);
}

static void phase_boost(void)
{
    int32_t start_max = 0;

    t.elapsed_ms += TASK_PERIOD_MS;
    if (t.elapsed_ms < BOOST_MS)
    {
        return;
    }

    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        if (w[i].sign != 0)
        {
            w[i].consec = 0;
            if (w[i].start_pwm > start_max) start_max = w[i].start_pwm;
        }
    }

    /*
     * El barrido grueso baja desde pwm_start. Los finos saltan directo a un
     * poco por encima del pwm_stop grueso (la rueda ya viene girando por el
     * BOOST, asi que sigue en regimen dinamico) para no recorrer toda la
     * zona entre arranque y parada a paso fino.
     */
    t.pwm = start_max;
    if (t.trial == 0)
    {
        t.step = COARSE_STEP;
    }
    else
    {
        t.step = FINE_STEP;
        if (t.ref_stop_max >= 0 && (t.ref_stop_max + FINE_MARGIN) < start_max)
        {
            t.pwm = t.ref_stop_max + FINE_MARGIN;
        }
    }

    apply_pwm(t.pwm);
    enter_state(PWMTEST_ST_RAMP_DOWN);
}

static void phase_ramp_down(void)
{
    uint8_t all_stopped = 1U;

    if (!step_tick())
    {
        return;
    }

    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        if (w[i].sign == 0 || w[i].stopped)
        {
            continue;
        }

        if (is_moving(i))
        {
            w[i].moved_in_descent = 1U;
            w[i].stop_pwm = t.pwm;
            w[i].stop_vel = w[i].vel;
            w[i].consec = 0;
        }
        else
        {
            w[i].consec++;
            if (w[i].consec >= CONFIRM_STEPS)
            {
                w[i].stopped = 1U;
                /*
                 * Si se detuvo ya en el primer escalon del descenso, el
                 * umbral real esta por encima del punto de partida y el
                 * valor no es confiable.
                 */
                if (w[i].moved_in_descent)
                {
                    w[i].flags |= PWMTEST_FLAG_STOP_OK;
                }
            }
        }

        if (!w[i].stopped)
        {
            all_stopped = 0U;
        }
    }

    emit_status();

    t.pwm -= t.step;
    if (all_stopped || t.pwm <= 0)
    {
        /* Si llego a 0 girando, el ultimo PWM con movimiento es el minimo. */
        for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
        {
            if (w[i].sign != 0 && !w[i].stopped && w[i].moved_in_descent)
            {
                w[i].flags |= PWMTEST_FLAG_STOP_OK;
            }
        }
        end_trial();
        return;
    }
    apply_pwm(t.pwm);
}

/* ========================================================================= */
/* Comandos                                                                  */
/* ========================================================================= */

static void start_command(const PwmTest_Cmd_t *cmd)
{
    uint8_t valid = 1U;

    if (cmd->mode == PWMTEST_MODE_ABORT)
    {
        finish_all(PWMTEST_ST_ABORTED);
        return;
    }

    if (cmd->mode == PWMTEST_MODE_SINGLE)
    {
        if ((cmd->target & 0x0F) == 0) valid = 0U;
    }
    else if (cmd->mode == PWMTEST_MODE_GROUP)
    {
        if (cmd->target < PWMTEST_PATTERN_FORWARD || cmd->target > PWMTEST_PATTERN_ROTATION) valid = 0U;
    }
    else
    {
        valid = 0U;
    }

    if (cmd->dir < -1 || cmd->dir > 1) valid = 0U;

    if (!valid)
    {
        finish_all(PWMTEST_ST_REJECTED);
        return;
    }

    t.mode = cmd->mode;
    t.mask = (cmd->mode == PWMTEST_MODE_SINGLE) ? (cmd->target & 0x0F) : 0x0F;
    t.pattern = (cmd->mode == PWMTEST_MODE_GROUP) ? cmd->target : 0;

    t.trials = cmd->trials;
    if (t.trials <= 0) t.trials = DEFAULT_TRIALS;
    if (t.trials > MAX_TRIALS) t.trials = MAX_TRIALS;

    t.pwm_limit = cmd->pwm_limit;
    if (t.pwm_limit <= 0) t.pwm_limit = DEFAULT_PWM_LIMIT;
    if (t.pwm_limit < MIN_PWM_LIMIT) t.pwm_limit = MIN_PWM_LIMIT;
    if (t.pwm_limit > PWM_MAX) t.pwm_limit = PWM_MAX;

    if (cmd->dir == 0)
    {
        t.dirs[0] = +1;
        t.dirs[1] = -1;
        t.n_dirs = 2;
    }
    else
    {
        t.dirs[0] = (int8_t)cmd->dir;
        t.n_dirs = 1;
    }

    t.dir_idx = 0;
    t.trial = 0;
    t.ref_valid = 0U;
    t.wheel = (t.mode == PWMTEST_MODE_SINGLE) ? next_wheel_in_mask(0) : 0;

    set_led(1U);
    begin_trial();
}

/* ========================================================================= */
/* API                                                                       */
/* ========================================================================= */

void PwmTest_Init(void)
{
    stop_all_motors();

    memset(w, 0, sizeof(w));
    memset(&t, 0, sizeof(t));
    t.state = PWMTEST_ST_IDLE;

    for (uint32_t i = 0; i < PWMTEST_N_WHEELS; i++)
    {
        w[i].last_cnt = read_encoder_raw(i);
    }

    if (evt_queue == NULL)
    {
        evt_queue = xQueueCreate(EVT_QUEUE_LEN, sizeof(PwmTest_Event_t));
    }
}

void PwmTest_Command(const PwmTest_Cmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    cmd_buffer = *cmd;
    cmd_pending = 1U;
    taskEXIT_CRITICAL();
}

uint8_t PwmTest_PopEvent(PwmTest_Event_t *evt)
{
    if (evt_queue == NULL || evt == NULL)
    {
        return 0U;
    }
    return (xQueueReceive(evt_queue, evt, 0) == pdTRUE) ? 1U : 0U;
}

void PwmTest_Update(void)
{
    PwmTest_Cmd_t cmd;
    uint8_t has_cmd = 0U;

    update_encoders();

    taskENTER_CRITICAL();
    if (cmd_pending)
    {
        cmd = cmd_buffer;
        cmd_pending = 0U;
        has_cmd = 1U;
    }
    taskEXIT_CRITICAL();

    if (has_cmd)
    {
        start_command(&cmd);
        return;
    }

    switch (t.state)
    {
        case PWMTEST_ST_REST:      phase_rest();      break;
        case PWMTEST_ST_RAMP_UP:   phase_ramp_up();   break;
        case PWMTEST_ST_BOOST:     phase_boost();     break;
        case PWMTEST_ST_RAMP_DOWN: phase_ramp_down(); break;

        default:
            /* Inactivo: latido periodico para que el host sepa que la STM vive. */
            t.idle_ms += TASK_PERIOD_MS;
            if (t.idle_ms >= IDLE_STATUS_MS)
            {
                t.idle_ms = 0U;
                emit_status();
            }
            break;
    }
}
