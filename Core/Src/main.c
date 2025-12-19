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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <SpeedControl.h>

#include "motor.h"
#include "Encoder.h"
#include "Buttan.h"
#include "FullColorLED.h"
#include "RunningSection.h"
#include "TrackingPart.h"
#include "TrackingSensor.h"
#include "IMU20649.h"
#include "log.h"
#include "flash2.h"

extern DMA_HandleTypeDef hdma_adc1; // DMAハンドルのextern宣言を追加
extern SPI_HandleTypeDef hspi3;
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

/* USER CODE BEGIN PV */
uint32_t adc_values[ADC_NUM]; // ADC値の配列を有効化
// uint32_t white_ref[ADC_CHANNEL_COUNT];
// uint32_t black_ref[ADC_CHANNEL_COUNT];
// uint16_t scaled_values[ADC_CHANNEL_COUNT];
uint32_t timer, timer1, timer2;

bool calibration_mode = true; // キャリブレーション中フラグ
int lion = 0;				  // mode変数を初期化
int bayado = -1;
bool watchdog_reset_detected = false; // Watchdogリセット検出フラグ
int16_t gx, gy, gz;
uint16_t cnt, cnt2 = 0;
uint16_t sw, sw2 = 0;

extern bool Start_Flag;
extern bool Stop_Flag;
extern int8_t trace_flag;
extern uint8_t Marker_State;
extern volatile uint32_t time_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// reo
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{ // 1ms
		timer++;
		timer2++;

		Encoder_Update();
		Sensor_Update();

		ControlLineTracking();
		TraceFlip();
		motorCtrlFlip();
		Fan_Ctrl();
		if (trace_flag)
		{
			Log_CalculateAndSave();
		}

		//		updateSideSensorStatus();
		read_gyro_data();  // グローバル変数 xg, yg, zg を更新
		read_accel_data(); // グローバル変数 xa, ya, za を更新
						   //		S_Sensor();
	}
	if (htim->Instance == TIM7)
	{ // 0.1ms
		timer1++;

		StorageBuffer();
		CourseOut();
		//		CourseOut();

		//		Encoder_Update();
		//				Sensor_Update();
		//
		//				ControlLineTracking();
		//				TraceFlip();
		//				runningFlip();
		//				motorCtrlFlip();
		//				updateSideSensorStatus();
		//
		//						CourseOut();
	}
}
void Init(void)
{
	// Watchdogリセットの検出
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
	{
		watchdog_reset_detected = true;
		__HAL_RCC_CLEAR_RESET_FLAGS(); // リセットフラグをクリア
	}

	// 長い待機中もWatchdogリフレッシュ（誤判定防止）
	for (int i = 0; i < 20; i++)
	{
		HAL_Delay(100);
		if (IWDG->SR == 0)
			IWDG->KR = 0xAAAA; // Watchdog有効なら定期的にリフレッシュ
	}
	LED(LED_RED);
	ADC_Init();
	Encoder_Init();
	HAL_TIM_Base_Start_IT(&htim6); // 1msタイマ開始
	HAL_TIM_Base_Start_IT(&htim7); // 1msタイマ開始
	Motor_Init();
	IMU_Init();
	Log_Init();
	IMU_CalibrateGyro();
	LED(LED_BLUE);
	LED(LED_WHITE);

	// Watchdogリセット後は安全のためキャリブレーションをスキップ
	if (!watchdog_reset_detected)
	{
		Calibration(); // キャリブレーションモード
	}
	else
	{
		// フリーズから復帰したことを視覚的に通知
		for (int i = 0; i < 5; i++)
		{
			LED(LED_RED);
			HAL_Delay(100);
			LED(LED_OFF);
			HAL_Delay(100);
		}
	}

	LED(LED_BLUE);
	// 初期化後の待機時間（Watchdogリフレッシュ付き）
	for (int i = 0; i < 10; i++)
	{
		HAL_Delay(100);
		if (IWDG->SR == 0)
			IWDG->KR = 0xAAAA;
	}

	// Independent Watchdog の初期化（完全に無効化）
	// 予期しないリセットを防ぐため、Watchdogは無効化
	// LSI = 32kHz, Prescaler = 64 → 約500Hz
	// Reload = 750 → 1.5秒（誤判定防止のため1秒より余裕を持たせる）
	// IWDG->KR = 0x5555; // レジスタアクセス許可
	// IWDG->PR = 0x04;   // Prescaler = 64 (0x04)
	// IWDG->RLR = 750;   // Reload value = 750 (約1.5秒)
	// IWDG->KR = 0xCCCC; // Watchdog起動
	// IWDG->KR = 0xAAAA; // Watchdogリフレッシュ
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
	MX_TIM8_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM1_Init();
	MX_SPI3_Init();
	/* USER CODE BEGIN 2 */
	Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		// Watchdogをリフレッシュ（trace_flag==1の時のみ有効）
		if (trace_flag == 1)
		{
			IWDG->KR = 0xAAAA;
		}

		// Watchdogリセット後は自動的に停止モードへ
		if (watchdog_reset_detected && bayado == -1)
		{
			bayado = 6; // 停止モード
			watchdog_reset_detected = false;
			//			printf("[WARNING] Watchdog reset detected. System stopped safely.\r\n");
		}

		//		// --- 右ボタン: lionをCaseとして確定 ---
		if (StatusR('R') == 2 && sw2 == 0)
		{
			timer2 = 0;
			sw2 = 1;
		}
		if (StatusR('R') == 2 && timer2 > 20 && sw2 == 1)
		{
			sw2 = 2;
		}
		if (timer2 > 40 && sw2 == 1)
		{
			sw2 = 0;
		}
		if (StatusR('R') == 0 && sw2 == 2)
		{
			Marker_State = 0;
			Start_Flag = false;
			Stop_Flag = false;
			timer2 = 0;
			// 3秒待機中もWatchdogリフレッシュ（誤判定防止）
			for (int i = 0; i < 30; i++)
			{
				HAL_Delay(100);
				IWDG->KR = 0xAAAA;
			}
			bayado = lion; // モードを確定！

			sw2 = 0;
		}
		if (StatusL('L') == 1 && sw == 0)
		{
			timer = 0;
			sw = 1;
		}
		if (StatusL('L') == 1 && timer > 20 && sw == 1)
		{
			sw = 2; // 長押し中
		}
		if (timer > 40 && sw == 1)
		{
			sw = 0; // チャタリング防止
		}
		if (StatusL('L') == 0 && sw == 2)
		{
			lion++;
			if (lion >= 8)
				lion = 0;
			sw = 0;
		}

		switch (lion)
		{
		case 0:
			LED(LED_RED);
			break;
		case 1:
			LED(LED_BLUE);
			break;
		case 2:
			LED(LED_GREEN);
			break;
		case 3:
			LED(LED_CYAN);
			break;
		case 4:
			LED(LED_MAGENTA);
			break;
		case 5:
			LED(LED_YELLOW);
			break;
		case 6:
			LED(LED_WHITE);
			break;
		case 7:
			LED(LED_OFF);
			stopTracking();
			break;
		default:
			break;
		}

		switch (bayado)
		{
		case 0:
			//			Log_Erase();
			//			bayado = -1;
			break;
		case 1:
			//			FanMotor(4000);
			//			if (timer2 >= 6000) {
			//				setTarget(0.9);
			//				startTracking(); //cyan
			//				S_Sensor();
			//			}
			printf("Gyro X: %d, Y: %d, Z: %d\r\n", xg, yg, zg);
			printf("Accel X: %d, Y: %d, Z: %d\r\n", xa, ya, za);
			//			FanMotor(4000);
			break;
		case 2:
			printf("Mode 2: Writing data...\r\n");
			WriteData();
			bayado = -1;
			break;
		case 3:
			//			FanMotor(4000);
			//			if (timer2 >= 6000) {
			//				setTarget(1.8);
			//				startTracking(); //cyan
			//				S_Sensor();
			Log_PrintData_To_Serial();
			//			Log_Test_Read_And_Print();
			bayado = -1; // 1回実行したらモードを終了

			break;
		case 4:
			FanMotor(4000);
			if (timer2 >= 6000)
			{
				setTarget(0.9);
				startTracking(); // cyan
				S_Sensor();

				// 現在の左右速度とエンコーダ値を表示
//				float current_speed_l, current_speed_r;
//				int16_t enc_l, enc_r;
//				getCurrentVelocity(&current_speed_l, &current_speed_r);
//				getEncoderCnt(&enc_l, &enc_r);
//				extern int16_t mon_rev_l, mon_rev_r;
//				float center_speed = (current_speed_l + current_speed_r) / 2.0f;
//				printf("L:%.3f R:%.3f | Center:%.3f | EncL:%d EncR:%d | PWML:%d PWMR:%d | Target:%.1f\r\n",
//					   current_speed_l, current_speed_r, center_speed, enc_l, enc_r, mon_rev_l, mon_rev_r, getTarget());
			}
			break;
		case 5:
			FanMotor(4000);
			if (timer2 >= 6000)
			{
				setTarget(2.0);
				startTracking(); // cyan
				S_Sensor();

				// 現在の左右速度とエンコーダ値を表示
				//							float current_speed_l, current_speed_r;
				//							int16_t enc_l, enc_r;
				//							getCurrentVelocity(&current_speed_l, &current_speed_r);
				//							getEncoderCnt(&enc_l, &enc_r);
				//							extern int16_t mon_rev_l, mon_rev_r;
				//							float center_speed = (current_speed_l + current_speed_r) / 2.0f;
				//							printf("L:%.3f R:%.3f | Center:%.3f | EncL:%d EncR:%d | PWML:%d PWMR:%d | Target:%.1f\r\n",
				//								   current_speed_l, current_speed_r, center_speed, enc_l, enc_r, mon_rev_l, mon_rev_r, getTarget());
			}
			break;
		case 6:
			stopTracking();
			setMotor(0, 0);
			FanMotor(0);
			break;
		case 7:
			eraseFlash();
			printf("Flash memory erased.\r\n");
			bayado = -1;
			break;
		default:

			break;
		}
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
