/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Logger.h"
#include "Buzzer.h"
#include "Buzzer_Melodias.h"
#include "Motovibrador.h"
#include "Flash.h"
#include "Lora.h"
#include "GPS.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Tarea 5: direccion de prueba para Flash — ultimo sector, igual que Flash_Test() */
#define FLASH_TASK5_TEST_ADDR   0x7FF000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* Tarea 6: handle de LoRa enlazado a mano (sin Lora_Init(), ver nota abajo) */
static Lora_Handle_t hlora;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* ===================  TAREA 1: UART de RS485 (Logger)  =================
   * Objetivo original: confirmar que se puede escribir por el UART de RS485
   * usando Log_Print/Log_Printf.
   *
   * CAMBIO EN LA MIGRACION A STM32L433CCUx: este MCU no tiene UART4. Se
   * decidio con el equipo dejar MCU_485_TX/RX (PA0/PA1) como GPIO_Output
   * simple en el .ioc, solo reservados, sin funcionalidad UART por ahora
   * (RS485 real queda pendiente de una decision de hardware aparte). El
   * Logger se reasigno temporalmente al UART de Bluetooth (huart3, USART3,
   * PB10/PB11) mientras se define una solucion definitiva.
   *
   * NOTA: Logger y Bluetooth comparten huart3 — no correr esta prueba a la
   * vez que otra prueba de Bluetooth.
   */
//  Log_Init();
//  Log_Print("TEST", "Boot OK - Tarea 1: UART Bluetooth (Logger, reasignado de RS485)");
  /* ======================================================================= */

  /* ===================  TAREA 2: Escaneo de bus I2C1  ====================
   * EN PAUSA: el escaneo de I2C en la tarjeta Mira dio problema de bus.
   * No se corre ninguna prueba de I2C en esta tarjeta hasta validar eso.
   *
   * Log_Print("TEST", "Boot OK - Tarea 2: Escaneo de bus I2C1");
   */
  /* ======================================================================= */

  /* ===================  TAREA 3: Buzzer (melodia)  =======================
   * Objetivo: confirmar el buzzer tocando una melodia completa una vez al
   * boot. TIM3 CH2 / PA7 (confirmar que este mapeo siga igual en L433).
   */
//  Buzzer_Init();
//  Log_Print("TEST", "Boot OK - Tarea 3: Buzzer - tocando melodia");
//  Buzzer_PlayMelody(ode_to_joy, MELODY_LEN(ode_to_joy), 120U);
//  Log_Print("TEST", "Tarea 3: Buzzer - fin de melodia");
  /* ======================================================================= */

  /* ===================  TAREA 4: Motovibrador  ============================
   * Objetivo: confirmar el motor ERM encendiendo 5 s al boot. GPIO PH0,
   * modo VIBRATOR_MODE_GPIO (on/off).
   */
//  Vibrator_Init();
//  Log_Print("TEST", "Boot OK - Tarea 4: Motovibrador - encendiendo 5s");
//  Vibrator_On();
//  HAL_Delay(5000U);
//  Vibrator_Off();
//  Log_Print("TEST", "Tarea 4: Motovibrador - apagado");
  /* ======================================================================= */

  /* ===================  TAREA 5: Flash SPI (MX25L6445E)  ==================
   * Objetivo: escribir "MENSAJE EN LA FLASH DE PRUEBA" una vez al boot y
   * despues, cada 3 s en el loop, releerlo y mandarlo por el Logger.
   * SPI2, CS manual PB5 (confirmar mapeo en L433).
   */
//  {
//      const char *flash_test_msg = "MENSAJE EN LA FLASH DE PRUEBA";
//
//      FlashStatus_e st_init  = Flash_Init();
//      FlashStatus_e st_erase = Flash_EraseSector(FLASH_TASK5_TEST_ADDR);
//      FlashStatus_e st_write = Flash_Write(FLASH_TASK5_TEST_ADDR,
//                                            (const uint8_t *)flash_test_msg,
//                                            (uint32_t)(strlen(flash_test_msg) + 1U));
//
//      Log_Printf("FLASH", "Init:%d Erase:%d Write:%d",
//                 (int)st_init, (int)st_erase, (int)st_write);
//  }
  /* ======================================================================= */

  /* ===================  TAREA 6: LoRa (RM1262) - AT por poleo  ============
   * Objetivo: mandar "AT\r\n" cada 3 s y confirmar que el modulo responde
   * "OK". Por POLEO (bloqueante), USART2/huart2 sin interrupcion.
   * NO llamamos Lora_Init() porque esa arma HAL_UART_Receive_IT() y deja el
   * huart en estado Busy_Rx, lo que bloquearia nuestras llamadas
   * bloqueantes de abajo. Solo enlazamos el handle a mano.
   */
//  hlora.huart = LORA_UART;
//  Log_Print("TEST", "Boot OK - Tarea 6: LoRa (AT) - por poleo");
  /* ======================================================================= */

  /* ===================  TAREA 7: GPS (L86-M33) - lectura cruda por poleo ===
   * El GPS no usa comandos AT: en cuanto FORCE_ON esta en HIGH transmite
   * solo tramas NMEA ($GPRMC, $GPGGA, ...) sin que se le pida nada. Aqui
   * solo confirmamos comunicacion leyendo bytes crudos del UART, sin
   * parsear todavia. Por POLEO, USART1/huart1 sin interrupcion.
   * NO llamamos Gps_Init() porque esa arma HAL_UART_Receive_IT() y deja el
   * huart en estado Busy_Rx, lo que bloquearia nuestras llamadas
   * bloqueantes de abajo. Gps_ForceOn() si se puede llamar: es solo un
   * HAL_GPIO_WritePin, no toca el UART.
   */
//  Gps_ForceOn();
//  Log_Print("TEST", "Boot OK - Tarea 7: GPS - lectura cruda por poleo");
  /* ======================================================================= */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ===================  TAREA 1: UART de RS485 (Logger)  =================
     * Reasignado a Bluetooth (huart3) — ver nota en USER CODE 2.
     */
//    Log_Print("TEST", "Escribiendo por UART3 (Bluetooth, Logger reasignado de RS485)");
//    HAL_Delay(1000U);
    /* ======================================================================= */

    /* ===================  TAREA 1C: Prueba directa por HAL en los UART  ====
     * Manda una cadena distinta por cada UART, directo por HAL, sin pasar
     * por el Logger — util para repetir en el proyecto nuevo si algun UART
     * sigue sin responder.
     *
     * CAMBIO: esta tarjeta (STM32L433CCUx) solo tiene 3 UART disponibles
     * (USART1=GPS, USART2=LoRa, USART3=Bluetooth) — no hay UART4/RS485, asi
     * que se quito msg4/huart4 respecto a la version original.
     */
//    const uint8_t msg1[] = "PRUEBA UART1\r\n";
//    const uint8_t msg2[] = "PRUEBA UART2\r\n";
//    const uint8_t msg3[] = "PRUEBA UART3\r\n";
//
//    HAL_UART_Transmit(&huart1, msg1, sizeof(msg1) - 1U, 100U);
//    HAL_Delay(500U);
//    HAL_UART_Transmit(&huart2, msg2, sizeof(msg2) - 1U, 100U);
//    HAL_Delay(500U);
//    HAL_UART_Transmit(&huart3, msg3, sizeof(msg3) - 1U, 100U);
    /* ======================================================================= */

    /* ===================  DIAGNOSTICO: Toggle GPIO puro (PH0, Motovibrador)
     * Confirma que el firmware SI esta corriendo en el micro, sin usar
     * ningun UART. Prende/apaga PH0 cada 4 s. Medir con multimetro: debe
     * alternar 3.3V / 0V. Util para repetir primero en el proyecto nuevo
     * antes que cualquier prueba de UART.
     */
//    HAL_GPIO_TogglePin(Motovibrador_GPIO_Port, Motovibrador_Pin);
//    HAL_Delay(4000U);
    /* ======================================================================= */

    /* ===================  TAREA 2: Escaneo de bus I2C1  ====================
     * EN PAUSA: el escaneo de I2C en la tarjeta Mira dio problema de bus.
     * No se corre hasta validar eso.
     */
//    {
//        uint8_t found      = 0U;
//        uint8_t busy_count = 0U;
//        uint8_t err_count  = 0U;
//
//        for (uint8_t addr = 1U; addr < 127U; addr++) {
//            HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1U, 10U);
//
//            if (st == HAL_OK) {
//                Log_Printf("I2C", "Dispositivo encontrado en 0x%02X", addr);
//                found++;
//            } else if (st == HAL_BUSY) {
//                busy_count++;
//            } else {
//                err_count++;
//            }
//        }
//
//        if (found > 0U) {
//            Log_Printf("I2C", "Total encontrados: %u", found);
//        } else if (busy_count == 126U) {
//            Log_Print("I2C", "Escaneo fallo: bus ocupado/atascado en todas las direcciones");
//        } else {
//            Log_Printf("I2C", "Ningun dispositivo encontrado (busy:%u err:%u de 126)",
//                       busy_count, err_count);
//        }
//
//        HAL_Delay(2000U);
//    }
    /* ======================================================================= */

    /* ===================  TAREA 3: Buzzer (melodia)  =======================
     * Sin loop propio — la melodia se reproducia una vez en USER CODE 2.
     */
//    HAL_Delay(2000U);
    /* ======================================================================= */

    /* ===================  TAREA 4: Motovibrador  ============================
     */
//    Log_Print("TEST", "Boot OK - Tarea 4: Motovibrador - encendiendo 5s");
//    Vibrator_On();
//    HAL_Delay(4000U);
//    Vibrator_Off();
//    Log_Print("TEST", "Tarea 4: Motovibrador - apagado");
//    HAL_Delay(4000U);
    /* ======================================================================= */

    /* ===================  TAREA 5: Flash SPI (MX25L6445E)  ==================
     */
//    char read_buf[64] = {0};
//
//    FlashStatus_e st_read = Flash_Read(FLASH_TASK5_TEST_ADDR,
//                                        (uint8_t *)read_buf,
//                                        sizeof(read_buf) - 1U);
//
//    Log_Printf("FLASH", "Read:%d Msg:\"%s\"", (int)st_read, read_buf);
//
//    HAL_Delay(3000U);
    /* ======================================================================= */

    /* ===================  TAREA 6: LoRa (RM1262) - AT por poleo  ============
     */
//    const uint8_t cmd[] = "AT\r\n";
//    uint8_t       resp[64] = {0};
//
//    HAL_UART_Transmit(hlora.huart, cmd, sizeof(cmd) - 1U, LORA_TX_TIMEOUT_MS);
//
//    HAL_StatusTypeDef st = HAL_UART_Receive(hlora.huart, resp, sizeof(resp) - 1U, LORA_RX_TIMEOUT_MS);
//
//    if (strstr((const char *)resp, "OK") != NULL) {
//        Log_Printf("LORA", "Respuesta OK: \"%s\"", (const char *)resp);
//    } else {
//        Log_Printf("LORA", "Sin OK (st:%d) resp:\"%s\"", (int)st, (const char *)resp);
//    }
//
//    HAL_Delay(3000U);
    /* ======================================================================= */

    /* ===================  TAREA 7: GPS (L86-M33) - lectura cruda por poleo ===
     * EN PAUSA: usa el mismo huart1 que la Tarea 1B (Logger). No correr
     * ambas a la vez.
     */
//    uint8_t raw[64] = {0};
//
//    HAL_StatusTypeDef st = HAL_UART_Receive(GPS_UART, raw, sizeof(raw) - 1U, 3000U);
//
//    if (st == HAL_OK || raw[0] != 0U) {
//        Log_Printf("GPS", "Crudo recibido: \"%s\"", (const char *)raw);
//    } else {
//        Log_Print("GPS", "Sin datos (timeout) - revisar FORCE_ON/cableado");
//    }
    /* ======================================================================= */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motovibrador_GPIO_Port, Motovibrador_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_485_TX_Pin|MCU_485_RX_Pin|GPS_FORCE_Pin|RS485_CONTROL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEL_PROG_LORA_o_BLE_Pin|SEL_PROG_MCU_o_ModFTDI_Pin|LORA_BOOT_Pin|BLE_AUTORUN_Pin
                          |BLE_VSP_Pin|CD_FLASH_Pin|LORA_RESET_Pin|BLE_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Motovibrador_Pin */
  GPIO_InitStruct.Pin = Motovibrador_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motovibrador_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_IMU_Pin SENSOR_IR1_Pin */
  GPIO_InitStruct.Pin = INT_IMU_Pin|SENSOR_IR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_485_TX_Pin MCU_485_RX_Pin GPS_FORCE_Pin RS485_CONTROL_Pin */
  GPIO_InitStruct.Pin = MCU_485_TX_Pin|MCU_485_RX_Pin|GPS_FORCE_Pin|RS485_CONTROL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_ID_Pin BLUETOOTH_MCU_Pin */
  GPIO_InitStruct.Pin = USB_ID_Pin|BLUETOOTH_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_IR2_Pin */
  GPIO_InitStruct.Pin = SENSOR_IR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_IR2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL_PROG_LORA_o_BLE_Pin SEL_PROG_MCU_o_ModFTDI_Pin LORA_BOOT_Pin BLE_AUTORUN_Pin
                           BLE_VSP_Pin CD_FLASH_Pin LORA_RESET_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = SEL_PROG_LORA_o_BLE_Pin|SEL_PROG_MCU_o_ModFTDI_Pin|LORA_BOOT_Pin|BLE_AUTORUN_Pin
                          |BLE_VSP_Pin|CD_FLASH_Pin|LORA_RESET_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
