/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Punto de entrada de la prueba de PWM minimo.
  *
  * Mismo hardware que stm_unified (4 ruedas FR, FL, BR, BL con encoder y
  * puente H). En vez del lazo PID corre una prueba que barre el PWM de cada
  * motor para encontrar el minimo con el que arranca y el minimo con el que
  * se sigue moviendo (ver Core/Src/pwm_test.c). main.c deja listo el
  * hardware basico y entrega la ejecucion a FreeRTOS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* No se requieren tipos privados extra en este archivo. */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* No se definen constantes privadas adicionales en este archivo. */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* No se definen macros privadas adicionales en este archivo. */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* No hay variables globales privadas adicionales en main.c. */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* El arranque de motores/encoders vive en freertos.c::StartMotorsAndEncoders,
 * justo antes de crear las tareas (ver MX_FREERTOS_Init). */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Punto de entrada de la aplicacion.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* No se usa inicializacion temprana adicional antes de HAL_Init(). */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /*
   * Reinicia perifericos, configura la capa HAL y deja activa la base de tiempo
   * minima del sistema. A partir de aqui se pueden usar funciones HAL.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* No se requiere inicializacion manual adicional antes del reloj del sistema. */
  /* USER CODE END Init */

  /* Configura PLL, buses y frecuencia central del microcontrolador. */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* No hay configuraciones de sistema adicionales fuera de CubeMX. */
  /* USER CODE END SysInit */

  /*
   * Inicializa todos los perifericos que usa esta STM:
   * - GPIO para enables (ENA/ENB), LED y señales digitales.
   * - DMA para transporte eficiente de micro-ROS por UART.
   * - USART2 para el transporte serie de micro-ROS.
   * - TIM1/TIM3/TIM5/TIM8 como encoders de las 4 ruedas (FR/FL/BR/BL).
   * - TIM2/TIM12/TIM10/TIM13 como PWM de los 4 motores.
   * - TIM6 como base de tiempo de HAL (ver stm32f4xx_hal_timebase_tim.c).
   * - I2C1 (IMU): se mantiene inicializado como en stm_unified, sin uso aqui.
   */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  /*
   * El arranque fisico de motores/encoders y la inicializacion de la prueba
   * (PwmTest_Init) se hacen dentro de MX_FREERTOS_Init(), justo antes de
   * crear tareas.
   */
  /* USER CODE END 2 */

  /* Inicializa el kernel CMSIS-RTOS2/FreeRTOS. */
  osKernelInitialize();
  MX_FREERTOS_Init();

  /* Entrega el control al scheduler; desde aqui corren las tareas. */
  osKernelStart();

  /*
   * En operacion normal no se deberia volver a este while, porque el scheduler
   * toma el control permanente del flujo.
   */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Zona de seguridad vacia. */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Configuracion del reloj del sistema.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*
   * Activa el reloj del bloque de potencia interno y selecciona la escala de
   * voltaje necesaria para operar a alta frecuencia.
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*
   * Usa HSE en modo bypass como fuente externa y construye el SYSCLK mediante
   * PLL. Los divisores elegidos llevan la MCU a 180 MHz.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* OverDrive es necesario para sostener la frecuencia maxima en este F4. */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Distribuye el reloj hacia CPU, AHB y periféricos APB1/APB2. Esta
   * configuracion impacta directamente timers, UART, I2C y latencias.
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#include <sys/time.h>

int _gettimeofday(struct timeval *tv, void *tzvp)
{
  /*
   * micro-ROS y parte de la libreria C esperan una referencia temporal POSIX.
   * Esta implementacion la aproxima usando HAL_GetTick(), que entrega
   * milisegundos desde el arranque.
   */
  (void)tzvp;
  uint32_t tick = HAL_GetTick();
  tv->tv_sec = tick / 1000;
  tv->tv_usec = (tick % 1000) * 1000;
  return 0;
}
/* USER CODE END 4 */

/**
  * @brief  Callback de periodo vencido en modo no bloqueante.
  * @note   Se usa para mantener la base de tiempo HAL cuando TIM6 dispara.
  *         TIM1 ya no es la base de tiempo en este proyecto: esta libre para
  *         el encoder de la rueda FR (ver stm32f4xx_hal_timebase_tim.c).
  * @param  htim : timer que genero la interrupcion.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* Sin acciones extra antes del manejo de tick HAL. */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* Sin acciones extra despues del manejo de tick HAL. */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  Rutina de error fatal.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /*
   * Ante un fallo de inicializacion o configuracion se deshabilitan
   * interrupciones y se entra en bucle infinito para congelar el sistema en un
   * estado diagnosticable.
   */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reporta el archivo fuente y la linea donde fallo un assert.
  * @param  file: nombre del archivo.
  * @param  line: numero de linea.
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /*
   * Punto de extension para enviar mensajes de depuracion cuando se usa
   * USE_FULL_ASSERT. Se deja vacio para no alterar el comportamiento actual.
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
