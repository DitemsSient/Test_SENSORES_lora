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

#include "lora_kg200z.h"
#include "helpers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Se recibe:
// {numOrden, ID, equipo, alias, vidas, municion, tiempo, mac1, mac2}
uint8_t  numOrden = 255;
uint8_t  ID = 255;
char     equipo[15] = "Equipo_XX";
char     alias[15]  = "Jugador_XX";
uint8_t  vidas = 0;
uint16_t municiones = 0;
uint16_t tiempo = 0;
char     mac1[15] = "AABBCCDDEEFFGG";
char     mac2[15] = "AABBCCDDEEFFGG";

// Se envía
// {ID, numOrden, vidas, municion, batería, longitud, latitud, altitud, orientación, pasos, ack, timestamp)}
uint8_t  bateria = 100;
float    latitud  = 19.436478f;
float    longitud = -99.176669f;
float    altitud  = 200.3f;
uint16_t orientacion = 0;
uint16_t pasos = 0;
uint8_t  ack = 0;
uint32_t timestamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void actualizarDatosSimulados(void);

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Arranca la recepción por interrupción con detección de "idle" */
  Lora_ArmReceive();

  dbg_println("Configurando LoRa KG200Z...");
  while (!setupLoRa()) {
	  HAL_Delay(500);
  }
  dbg_println("LoRa listo, enlazando...");

  while (!connectLoRa()) {
	  HAL_Delay(1000);
  }

  char hexPayload[200];

  while (1) {
	  dbg_println("Esperando datos...");
	  if (sendAndCheckLora("AT+QSEND=1:1:FF", "SEND_CONFIRMED", 5000, 0)) { /* MAC rxDone */
		  if (esperarDatoLora(hexPayload, sizeof(hexPayload), 2000)) {
			  dbg_println("Payload recibido.");
			  dbg_println(hexPayload);

			  uint8_t decodedBytes[100];
			  size_t decodedLen = decodeHexToBytes(hexPayload, decodedBytes, sizeof(decodedBytes) - 1);
			  decodedBytes[decodedLen] = '\0';

			  dbg_print("Decodificado: ");
			  dbg_println((char *)decodedBytes);

			  if (parsearDatosLora((char *)decodedBytes)) {
				  dbg_println("Datos del jugador guardados correctamente.");

				  ack = 1;

				  if (generarCadena() && mandarPorLora()) {
					  dbg_println("ACK enviado.");
				  } else {
					  dbg_println("Error: ACK falló.");
				  }
				  break;
			  } else {
				  dbg_println("Error: formato de payload inesperado.");
			  }
		  }
	  } else dbg_println("Error: Timeout Lora");
	  HAL_Delay(1000);
  }

  HAL_Delay(1000);

  while (1) {
	  dbg_println("Esperando señal de inicio...");
	  if (sendAndCheckLora("AT+QSEND=1:1:FF", "SEND_CONFIRMED", 5000, 0)) { /* MAC rxDone */
		  if (loraResponse("41414141", 2000, 0)) { /* Código de inicio: AAAA (hex) */
			  dbg_println("# # # # # INICIO DE ENTRENAMIENTO # # # # #");
			  break;
		  }
	  }
	  HAL_Delay(1000);
  }

  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  actualizarDatosSimulados();

	  if (generarCadena() && mandarPorLora()) {
		  dbg_println("JSON enviado correctamente por LoRa.");

		  if (loraResponse("46464646", 1000, 0)) { /* Código de fin: FFFF (hex) */
			  dbg_println("# # # # # FIN DE ENTRENAMIENTO # # # # #");
			  while (1) {
				  HAL_Delay(1000);
			  }
		  }
	  } else {
		  dbg_println("Error: no se pudo enviar el JSON por LoRa.");
	  }

	  HAL_Delay(5000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void actualizarDatosSimulados(void)
{
	/* Simular cambio de variables */
    if (municiones > 0) municiones--;        /* valor fijo */
    if (bateria > 0)    bateria--;            /* valor fijo */

    latitud  += 0.00001f;                     /* valor fijo */
    longitud += 0.00001f;                     /* valor fijo */
    altitud  += 0.1f;                         /* valor fijo */

    orientacion = (uint16_t)((orientacion + 1) % 360);  /* valor fijo */
    pasos += 1;                               /* valor fijo */

    timestamp++;
}

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
#ifdef USE_FULL_ASSERT
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
