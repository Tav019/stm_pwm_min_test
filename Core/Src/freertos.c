/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Tareas FreeRTOS de la prueba de PWM minimo.
  *
  * Mismo hardware y misma base que stm_unified (4 encoders, 4 PWM, FreeRTOS,
  * micro-ROS por USART2), pero en vez del lazo PID + odometria corre una
  * maquina de estados que barre el PWM de cada motor y mide con los encoders
  * a partir de que PWM la rueda arranca y hasta que PWM se sigue moviendo
  * (ver Core/Src/pwm_test.c).
  *
  * Interfaz micro-ROS:
  * - /pwm_test/cmd    (entrada, Int32MultiArray):
  *     [mode, target, dir, trials, pwm_limit]
  *       mode:      0 = abortar, 1 = una rueda a la vez, 2 = 4 ruedas juntas
  *       target:    modo 1 -> mascara de ruedas (1=FR, 2=FL, 4=BR, 8=BL, 15=todas)
  *                  modo 2 -> patron (0=avance, 1=lateral, 2=rotacion)
  *       dir:       +1, -1 o 0 (= ambos sentidos)
  *       trials:    repeticiones finas (opcional, por defecto 3)
  *       pwm_limit: PWM maximo en cuentas 0..999 (opcional, por defecto 600)
  * - /pwm_test/status (salida, Float64MultiArray, cada escalon):
  *     [state, mode, target, dir, trial, pwm, vel_fr, vel_fl, vel_br, vel_bl]
  * - /pwm_test/result (salida, Float64MultiArray, uno por rueda y intento):
  *     [mode, pattern, wheel, dir, trial, pwm_start, pwm_stop,
  *      vel_start, vel_stop, flags]
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "pwm_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* No se requieren tipos privados adicionales. */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;

/* Funciones del transporte serial micro-ROS basado en UART/DMA. */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

/* Allocadores adaptados a FreeRTOS. */
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#define RCCHECK(fn)      do { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { error_loop(); } } while (0)
#define RCSOFTCHECK(fn)  do { (void)(fn); } while (0)

/* Periodo de la tarea de prueba en milisegundos. */
#define TEST_TASK_PERIOD_MS          10U

/* Tamaño maximo del comando /pwm_test/cmd. */
#define CMD_MESSAGE_CAPACITY         8U

static void error_loop(void)
{
    /* Indicador visual de fallo severo; motores detenidos por seguridad. */
    PwmTest_Command(&(PwmTest_Cmd_t){ .mode = PWMTEST_MODE_ABORT });
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osDelay(100);
    }
}

static void StartMotorsAndEncoders(void)
{
    /*
     * Habilita los drivers de las 2 placas de puente H (ver nota de ENA/ENB
     * en main.h): una para FR/FL y otra para BR/BL, 4 pines en total.
     */
    HAL_GPIO_WritePin(ENA_FR_FL_GPIO_Port, ENA_FR_FL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENB_FR_FL_GPIO_Port, ENB_FR_FL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENA_BR_BL_GPIO_Port, ENA_BR_BL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENB_BR_BL_GPIO_Port, ENB_BR_BL_Pin, GPIO_PIN_SET);

    /* Arranca los encoders de las 4 ruedas. */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); /* FR */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); /* FL */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); /* BR */
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); /* BL */

    /* Arranca las salidas PWM asociadas a los 4 motores. */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  /* FR Adelante */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  /* FL Adelante */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  /* FL Atras */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  /* FR Atras */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); /* BR Adelante */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); /* BR Atras */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); /* BL Adelante */
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1); /* BL Atras */
}

/*
 * /pwm_test/cmd: [mode, target, dir, trials, pwm_limit]. Los dos ultimos
 * son opcionales; si faltan se usan los valores por defecto del modulo.
 */
static void pwm_test_cmd_callback(const void * msgin)
{
    const std_msgs__msg__Int32MultiArray * msg =
        (const std_msgs__msg__Int32MultiArray *)msgin;
    PwmTest_Cmd_t cmd = {0};

    if (msg->data.size < 1U)
    {
        return;
    }

    cmd.mode      = msg->data.data[0];
    cmd.target    = (msg->data.size > 1U) ? msg->data.data[1] : 0;
    cmd.dir       = (msg->data.size > 2U) ? msg->data.data[2] : 0;
    cmd.trials    = (msg->data.size > 3U) ? msg->data.data[3] : 0;
    cmd.pwm_limit = (msg->data.size > 4U) ? msg->data.data[4] : 0;

    PwmTest_Command(&cmd);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* No se requieren macros privadas adicionales. */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN Variables */
/* No hay variables globales adicionales fuera de la logica descrita arriba. */
/* USER CODE END Variables */

/* Tarea de la maquina de estados de la prueba (barrido de PWM + encoders). */
osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Tarea de comunicacion micro-ROS. */
osThreadId_t microRosTaskHandle;
const osThreadAttr_t microRosTask_attributes = {
  .name = "microRosTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void StartTestTask(void *argument);
static void StartMicroRosTask(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void);

/**
  * @brief  Inicializacion de FreeRTOS.
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* Se habilita primero el hardware de movimiento. */
  StartMotorsAndEncoders();

  /* Referencia los encoders ya arrancados y crea la cola de eventos. */
  PwmTest_Init();
  /* USER CODE END Init */

  testTaskHandle = osThreadNew(StartTestTask, NULL, &testTask_attributes);
  microRosTaskHandle = osThreadNew(StartMicroRosTask, NULL, &microRosTask_attributes);
}

static void StartTestTask(void *argument)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  (void)argument;

  for (;;)
  {
    PwmTest_Update();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TEST_TASK_PERIOD_MS));
  }
}

static void StartMicroRosTask(void *argument)
{
  rclc_executor_t executor;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rcl_subscription_t cmd_sub;
  rcl_publisher_t status_pub;
  rcl_publisher_t result_pub;
  PwmTest_Event_t evt;

  std_msgs__msg__Int32MultiArray cmd_msg = {0};
  std_msgs__msg__Float64MultiArray status_msg = {0};
  std_msgs__msg__Float64MultiArray result_msg = {0};

  /* Buffers estaticos para mensajes entrantes y salientes. */
  static int32_t cmd_buffer[CMD_MESSAGE_CAPACITY];
  static double status_buffer[PWMTEST_STATUS_SIZE];
  static double result_buffer[PWMTEST_RESULT_SIZE];

  (void)argument;

  rmw_uros_set_custom_transport(
    true,
    (void *)&huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
      error_loop();
  }

  cmd_msg.data.capacity = CMD_MESSAGE_CAPACITY;
  cmd_msg.data.size = 0;
  cmd_msg.data.data = cmd_buffer;

  status_msg.data.capacity = PWMTEST_STATUS_SIZE;
  status_msg.data.size = PWMTEST_STATUS_SIZE;
  status_msg.data.data = status_buffer;

  result_msg.data.capacity = PWMTEST_RESULT_SIZE;
  result_msg.data.size = PWMTEST_RESULT_SIZE;
  result_msg.data.data = result_buffer;

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "stm_pwm_min_test_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/pwm_test/cmd"));

  RCCHECK(rclc_publisher_init_default(
      &status_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/pwm_test/status"));

  RCCHECK(rclc_publisher_init_default(
      &result_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/pwm_test/result"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &pwm_test_cmd_callback, ON_NEW_DATA));

  for (;;)
  {
    /* Procesa comandos entrantes de forma no bloqueante. */
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    /* Publica todo lo que la tarea de prueba haya generado desde la ultima vuelta. */
    while (PwmTest_PopEvent(&evt))
    {
      if (evt.type == PWMTEST_EVT_RESULT)
      {
        for (uint32_t i = 0; i < PWMTEST_RESULT_SIZE; i++)
        {
          result_msg.data.data[i] = (double)evt.data[i];
        }
        RCSOFTCHECK(rcl_publish(&result_pub, &result_msg, NULL));
      }
      else
      {
        for (uint32_t i = 0; i < PWMTEST_STATUS_SIZE; i++)
        {
          status_msg.data.data[i] = (double)evt.data[i];
        }
        RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
      }
    }
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* No se agrega codigo de aplicacion adicional en este archivo. */
/* USER CODE END Application */
