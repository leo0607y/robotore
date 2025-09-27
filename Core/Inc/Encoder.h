/**
 * @brief 左右ホイールの累積移動距離[mm]を取得
 * @param dist_l 左ホイール移動距離格納先
 * @param dist_r 右ホイール移動距離格納先
 */
#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define PI 3.1415926535f
#define WHEEL_RADIUS (11.875f / 1000.0f) // [m] ホイール半径
//#define ENCODER_RESOLUTION 4096.0f                                                         // エンコーダ分解能
#define ENCODER_RESOLUTION 4096.0f
#define REDUCTION_RATIO (18.0f / 64.0f)                                                           // 減速比
#define DISTANCE_PER_CNT (2.0f * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[mm/cnt]
#define CNT_OFFSET 32768

extern int32_t enc_l_total;
extern int32_t enc_r_total;

void getWheelDistance(float *dist_l, float *dist_r);

/**
 * @brief エンコーダ初期化（タイマ・カウンタリセット）
 */
void Encoder_Init(void);

/**
 * @brief エンコーダカウント値の更新・距離積算
 *        1ms周期などで呼び出し、走行距離・判定用距離を加算
 */
void Encoder_Update(void);

/**
 * @brief 現在の左右エンコーダカウント値を取得
 * @param cnt_l 左カウント格納先ポインタ
 * @param cnt_r 右カウント格納先ポインタ
 */
void getEncoderCnt(int16_t *cnt_l, int16_t *cnt_r);

/**
 * @brief 現在の左右エンコーダ累積カウント値を取得（0～65535の正値で返す）
 * @param total_l 左累積カウント格納先ポインタ
 * @param total_r 右累積カウント格納先ポインタ
 */
void getEncoderTotal(int16_t *total_l, int16_t *total_r);

// --- 距離・判定用API ---
float getDistance(void);            // 10mm単位の距離取得
void clearDistance(void);           // 10mm距離リセット
float getCrossLineIgnoreDistance(void); // クロスライン無視距離取得
float getSideLineIgnoreDistance(void);  // サイドライン無視距離取得
float getTotalDistance();               // 総走行距離取得
float getGoalJudgeDistance();           // ゴール判定用距離取得
float getSideLineJudgeDistance();       // サイドライン判定用距離取得
float getspeedcount();                  // 速度カウント取得
void clearspeedcount(void);             // 速度カウントリセット

void setTotalDistance(float distance); // 総走行距離セット

void clearCrossLineIgnoreDistance(void); // クロスライン無視距離リセット
void clearSideLineIgnoreDistance(void);  // サイドライン無視距離リセット
void resetEncoderCnt(void);              // エンコーダカウントリセット
void clearTotalDistance();               // 総走行距離リセット
void clearGoalJudgeDistance();           // ゴール判定距離リセット
void clearSideLineJudgeDistance();       // サイドライン判定距離リセット

#endif /* INC_ENCODER_H_ */
