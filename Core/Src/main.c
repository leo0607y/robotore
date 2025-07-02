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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNEL_COUNT 12
#define PWM_MAX 3200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t adc_values[ADC_CHANNEL_COUNT];
uint32_t white_ref[ADC_CHANNEL_COUNT];
uint32_t black_ref[ADC_CHANNEL_COUNT];
uint16_t scaled_values[ADC_CHANNEL_COUNT];

bool calibration_mode = true;                     // キャリブレーション中フラグ

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_calibration_led(bool white_mode);
void wait_for_button_press(void);
void capture_reference(uint32_t ref[]);
void apply_calibration(uint32_t raw[], uint16_t scaled[]);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
float Kp = 0.48f;
float Ki = 0.0f;
float Kd = 0.0f;
int32_t integral = 0;
int32_t previous_error = 0;
int32_t dead_zone = 40;
int32_t base_speed = 150;
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
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  printf("Hello from UART!\r\n");

    if (HAL_ADC_Start_DMA(&hadc1, adc_values, ADC_CHANNEL_COUNT) != HAL_OK) {
        printf("HAL_ADC_Start_DMA FAILED (%ld)\r\n", HAL_ADC_GetError(&hadc1));
    } else {
        printf("ADC Start OK\r\n");
    }
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    // --- Calibration ---
    printf("Push button to record white reference (on white line)...\r\n");
    set_calibration_led(true);
    wait_for_button_press();
    capture_reference(white_ref);
    printf("White reference captured.\r\n");

    printf("Push button to record black reference (on black floor)...\r\n");
    set_calibration_led(false);
    wait_for_button_press();
    capture_reference(black_ref);
    printf("Black reference captured.\r\n");

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    //reset
    integral = 0;
    previous_error = 0;
    HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  apply_calibration(adc_values, scaled_values);

	      int32_t right_sum = scaled_values[0] + scaled_values[2]*1.2 + scaled_values[4]*1.3 +
	                          scaled_values[6]*1.5 + scaled_values[8] *1.6+ scaled_values[10]*1.2;
	      int32_t left_sum  = scaled_values[1] + scaled_values[3]*1.2 + scaled_values[5]*1.3 +
	                          scaled_values[7]*1.5 + scaled_values[9]*1.6 + scaled_values[11]*1.2;

	      int32_t error = (right_sum - left_sum) / 10;

	      integral += error;
	      if (integral > 1000) integral = 1000;
	      if (integral < -1000) integral = -1000;

	      float derivative = error - previous_error;
	      float control_f = Kp * error + Ki * integral + Kd * derivative;
	      previous_error = error;

	      int32_t control_pwm = (int32_t)control_f;

	      // --- デッドゾーン処理 ---
	      if (abs(control_pwm) < dead_zone) {
	          control_pwm = 0;
	      }

	      // --- 左右PWM値の計算 ---
	      int32_t right_pwm = base_speed + control_pwm;
	      int32_t left_pwm  = base_speed - control_pwm;

	      // --- PWM制限（クリッピング） ---
	      if (right_pwm > PWM_MAX) right_pwm = PWM_MAX;
	      if (right_pwm < 0)        right_pwm = 0;
	      if (left_pwm > PWM_MAX)   left_pwm = PWM_MAX;
	      if (left_pwm < 0)         left_pwm = 0;

	      // --- モータ制御 ---
	      // 正転指定（前進）
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);  // 右モータ正転
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // 左モータ正転

	      // PWM出力（前進）
	      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, right_pwm);  // 右PWM
	      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, left_pwm);   // 左PWM

	      // --- デバッグ出力 ---
//	      printf("error=%ld, L_pwm=%ld, R_pwm=%ld, control=%ld\r\n", error, left_pwm, right_pwm, control_pwm);


	      HAL_Delay(1);  // 10ms周期



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

void set_calibration_led(bool white_mode) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, white_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, white_mode ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void apply_calibration(uint32_t raw[], uint16_t scaled[]) {
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int32_t diff = (int32_t)black_ref[i] - (int32_t)white_ref[i];
        if (diff == 0) {
            scaled[i] = 0;
        } else {
            int32_t norm = (int32_t)raw[i] - (int32_t)white_ref[i];
            if (norm < 0) norm = 0;
            if (norm > diff) norm = diff;
//            scaled[i] = (uint16_t)((diff - norm) * 1023 / diff);
            scaled[i] = (uint16_t)(norm* 1023 / diff);
        }

    }
}

void wait_for_button_press(void) {
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET);
    HAL_Delay(50);
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET);
    HAL_Delay(50);
}

void capture_reference(uint32_t ref[]) {
    for (int i = 0; i < 20; i++) {
        toggle_calibration_led();
        HAL_Delay(100);
    }
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        ref[i] = adc_values[i];
    }
}

//PID制御　ライントレース




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
