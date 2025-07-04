#include "SpeedControl.h"
#include "Encoder.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>

// --- 内部変数 ---
static float target_speed_cm_s = 100.0f; // 目標速度 [cm/s]（デフォルト100cm/s=1m/s）
static float current_speed_cm_s = 0.0f;  // 現在速度 [cm/s]
static float integral = 0.0f;            // 積分項
static float prev_error = 0.0f;          // 前回の偏差

// --- PIDゲイン（調整が必要） ---
#define KP 10.0f
#define KI 0.5f
#define KD 0.0f

// --- 制御周期（秒） ---
#define CONTROL_DT 0.001f // 10ms周期想定

/**
 * @brief 速度制御モジュール初期化
 */
void SpeedControl_Init(void)
{
    integral = 0.0f;
    prev_error = 0.0f;
    current_speed_cm_s = 0.0f;
}

/**
 * @brief 目標速度（cm/s）を設定
 */
void SpeedControl_SetTargetSpeed(float speed)
{
    target_speed_cm_s = speed;
}

/**
 * @brief 現在速度（cm/s）を取得
 */
float SpeedControl_GetCurrentSpeed(void)
{
    return current_speed_cm_s;
}

/**
 * @brief 速度制御ループ（一定周期で呼び出し）
 *        エンコーダ値から現在速度を算出し、PID演算でモータ出力を決定
 */
void SpeedControl_Update(void)
{
    // --- 1. エンコーダから現在速度算出（両輪平均、10ms周期想定） ---
    int16_t cnt_l, cnt_r;
    getEncoderCnt(&cnt_l, &cnt_r);
    // Encoder.cのDISTANCE_PER_CNTを使い、1周期あたりの移動距離[mm]を計算
    // ここでは10ms周期想定なので速度[cm/s] = (距離[mm]/10) * 100
    float distance_mm = (cnt_l + cnt_r) / 2.0f * DISTANCE_PER_CNT;
    current_speed_cm_s = (distance_mm / CONTROL_DT) / 10.0f;

    // --- 2. PID制御演算 ---
    float error = target_speed_cm_s - current_speed_cm_s;
    integral += error * CONTROL_DT;
    float derivative = (error - prev_error) / CONTROL_DT;
    float output = KP * error + KI * integral + KD * derivative;
    prev_error = error;

    // --- 3. モータ出力に反映（左右同じ値を仮定） ---
    // スケーリングを大きく
    int16_t pwm = (int16_t)(output * 1.0f);
    // setMotorの範囲にクリップ
    if (pwm > 300)
        pwm = 300;
    if (pwm < -300)
        pwm = -300;
    setMotor(pwm, pwm);
//    printf("spd:%.2f cm/s,pwm:%d,cnt_l:%d,cnt_r:%d \n", current_speed_cm_s, pwm,
//           cnt_l, cnt_r);
}
