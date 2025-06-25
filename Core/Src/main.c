/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNEL_COUNT 12


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_values[ADC_CHANNEL_COUNT];          // 生値読み取り用
uint32_t max_values[ADC_CHANNEL_COUNT];          // 最大値保持
uint32_t min_values[ADC_CHANNEL_COUNT];          // 最小値保持
uint32_t calibration_start_time = 0;
uint32_t led_timer = 0;
uint16_t calibrated_values[ADC_CHANNEL_COUNT];


bool calibration_mode = true;                     // キャリブレーション中フラグ

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void toggle_calibration_led(void);
void calibrate_adc_values(void);
void apply_calibration(uint32_t raw[], uint16_t scaled[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);  // ← ここで1秒待つ（ADC安定化のため）
  printf("Start 10s calibration\r\n");

  HAL_ADC_Start_DMA(&hadc1, adc_values, ADC_CHANNEL_COUNT);

  // max/min初期化
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
      max_values[i] = 0;
      min_values[i] = 0xFFFFFFFF;
  }

  calibration_mode = true;
  uint32_t calibration_start_time = HAL_GetTick();
  uint32_t led_timer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (calibration_mode) {
          calibrate_adc_values();  // min/max 更新

          // LEDを500msごとにトグル
          if (HAL_GetTick() - led_timer >= 500) {
              led_timer = HAL_GetTick();
              toggle_calibration_led();
          }

          // 20秒でキャリブレーション終了
          if (HAL_GetTick() - calibration_start_time >= 10000) {
              calibration_mode = false;
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
              printf("Calibration finished!\r\n");

              // キャリブレーション結果確認出力
              printf("Min: ");
              for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
                  printf("%4lu ", min_values[i]);
              }
              printf("\r\n");

              printf("Max: ");
              for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
                  printf("%4lu ", max_values[i]);
              }
              printf("\r\n");
          }
      } else {
    	  apply_calibration(adc_values, calibrated_values);

          printf("Raw ADC: ");
          for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
              printf("%4lu ", adc_values[i]);
          }
          printf("\r\n");

          printf("Calibrated ADC: ");
          for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        	  printf("%4u ", calibrated_values[i]);

          }
          printf("\r\n");
      }

      HAL_Delay(200);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//キャリブレーション
void toggle_calibration_led(void) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
}

void calibrate_adc_values(void) {
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (adc_values[i] > max_values[i]) max_values[i] = adc_values[i];
        if (adc_values[i] < min_values[i]) min_values[i] = adc_values[i];
    }
}

void apply_calibration(uint32_t raw[], uint16_t scaled[]) {
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        uint32_t range = max_values[i] - min_values[i];
        if (range == 0) {
            scaled[i] = 0;
        } else {
            scaled[i] = (uint16_t)(1024.0f * (float)(raw[i] - min_values[i]) / (float)(range));
        }
    }
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
