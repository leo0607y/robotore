
#include "motor.h"
#include "tim.h"

// --- モータ制御用グローバル変数 ---
extern TIM_HandleTypeDef htim8; // メインモータ用タイマ CH1&CH3使用

static int16_t motor_l, motor_r; // 左右モータ出力値
int16_t rotation_l = 0;          // 左モータ回転量（エンコーダ用）
int16_t rotation_r = 0;          // 右モータ回転量（エンコーダ用）
int16_t mon_rev_l, mon_rev_r;    // デバッグ用: PWM値の監視

/**
 * @brief モータPWMの初期化・安全停止
 *        PWM出力開始＆全チャンネルを停止状態にセット
 *        （起動時の暴走防止）
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // 左モータPWM開始
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // 右モータPWM開始
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Fan
    HAL_Delay(100);                           // 安全のため少し待機
}

/**
 * @brief 左右モータの正転・逆転制御（PWM出力値を反映）
 *        motor_l, motor_rの値に応じてPWM出力・回転方向を切り替え
 *        GPIOで正転/逆転を制御
 */
void motorCtrlFlip(void)
{
    // 緊急停止ガード：trace_flag=0の場合はモーター停止を維持
    extern int8_t trace_flag;
    if (trace_flag == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0); // 左モーターPWM=0
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // 右モーターPWM=0
        return;
    }

    int16_t motor_pwm_l, motor_pwm_r;

    // 左モータ制御
    if (motor_l >= 0)
    {
        motor_pwm_l = motor_l;
        // 正転: PWM出力, GPIOで正転指定
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motor_pwm_l);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    }
    else
    {
        motor_pwm_l = motor_l * (-1);
        // 逆転: PWM出力, GPIOで逆転指定
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motor_pwm_l);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    }

    // 右モータ制御
    if (motor_r >= 0)
    {
        motor_pwm_r = motor_r;
        // 正転: PWM出力, GPIOで正転指定
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motor_pwm_r);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    else
    {
        motor_pwm_r = motor_r * (-1);
        // 逆転: PWM出力, GPIOで逆転指定
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motor_pwm_r);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }
    // デバッグ用: 実際に出力したPWM値を記録
    mon_rev_l = motor_pwm_l;
    mon_rev_r = motor_pwm_r;
}

void setMotor(int16_t l, int16_t r)
{
    // 緊急停止ガード：trace_flag=0の場合は強制的に0に設定
    extern int8_t trace_flag;
    if (trace_flag == 0)
    {
        motor_l = 0;
        motor_r = 0;
        return;
    }

    if (l >= MAX_COUNTER_PERIOD)
        l = MAX_COUNTER_PERIOD;
    else if (l <= MIN_COUNTER_PERIOD)
        l = MIN_COUNTER_PERIOD;

    if (r >= MAX_COUNTER_PERIOD)
        r = MAX_COUNTER_PERIOD;
    else if (r <= MIN_COUNTER_PERIOD)
        r = MIN_COUNTER_PERIOD;

    motor_l = l;
    motor_r = r;
}
