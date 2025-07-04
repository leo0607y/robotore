#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

#include <stdint.h>

/**
 * @brief 速度制御モジュール初期化
 */
void SpeedControl_Init(void);

/**
 * @brief 目標速度（cm/s）を設定
 * @param target_speed_cm_s 目標速度 [cm/s]
 */
void SpeedControl_SetTargetSpeed(float target_speed_cm_s);

/**
 * @brief 速度制御ループ（一定周期で呼び出し）
 *        エンコーダ値から現在速度を算出し、PID演算でモータ出力を決定
 */
void SpeedControl_Update(void);

/**
 * @brief 現在速度（cm/s）を取得
 */
float SpeedControl_GetCurrentSpeed(void);

#endif // SPEEDCONTROL_H
