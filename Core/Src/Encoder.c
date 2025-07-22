/**
 * @brief 左右ホイールの累積移動距離[mm]を取得
 * @param dist_l 左ホイール移動距離格納先
 * @param dist_r 右ホイール移動距離格納先
 */

#include "Encoder.h"
#include "tim.h"
//
//// --- エンコーダ・走行距離計算用定数 ---
// #define MAX_ENCODER_CNT 65535 // エンコーダカウンタ最大値
// #define CNT_OFFSET 10000      // カウンタオフセット値
//
// #define WHEEL_RADIUS 11.875 // [mm] ホイール半径
// #define PI 3.1415926535
// #define ENCODER_RESOLUTION 4096                                                         // エンコーダ分解能
// #define REDUCTION_RATIO 0.28125                                                         // 減速比
// #define DISTANCE_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[mm/cnt]

// --- タイマハンドル ---
// TIM_HandleTypeDef htim3; // 左エンコーダ用
// TIM_HandleTypeDef htim4; // 右エンコーダ用
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern int32_t enc_l_total;
extern int32_t enc_r_total;
int32_t enc_l_total = 0;
int32_t enc_r_total = 0;

// --- エンコーダカウント・距離管理用変数 ---
static int16_t enc_l_cnt, enc_r_cnt;     // カウント値・累積値
static float distance_1ms;               // 1msごとの移動距離
static float distance_10mm;              // 10mm単位の距離
static float sab_distance_10mm;          // サブ用10mm距離
static float total_distance;             // 総走行距離
static float goal_judge_distance;        // ゴール判定用距離
static float side_line_judge_distance;   // サイドライン判定用距離
static float distance_cross_line_ignore; // クロスライン無視距離
static float distance_side_line_ignore;  // サイドライン無視距離
static float speed_cnt;                  // 速度カウント
static int16_t enc_l_cnt = 0;
static int16_t enc_r_cnt = 0;

/**
 * @brief エンコーダ初期化（タイマ起動・カウンタリセット）
 */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 左エンコーダ開始
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 右エンコーダ開始
    TIM3->CNT = CNT_OFFSET;
    TIM4->CNT = CNT_OFFSET;
}

/**
 * @brief エンコーダカウント値の更新・距離積算
 *        1ms周期などで呼び出し、走行距離・判定用距離を加算
 */
void Encoder_Update(void)
{
    // 現在のカウント値を取得
    enc_l_cnt = TIM3->CNT - CNT_OFFSET;
    enc_r_cnt = CNT_OFFSET - TIM4->CNT;

    // 累積カウントに加算
    enc_l_total += enc_l_cnt;
    enc_r_total += enc_r_cnt;
    // 距離計算（両輪平均）
    distance_1ms = DISTANCE_PER_CNT * (enc_l_cnt + enc_r_cnt) / 2;
    distance_10mm += distance_1ms;
    sab_distance_10mm += distance_1ms;
    total_distance += distance_1ms;
    goal_judge_distance += distance_1ms;
    side_line_judge_distance += distance_1ms;
    distance_cross_line_ignore += distance_1ms;
    distance_side_line_ignore += distance_1ms;

    // カウンタリセット
    TIM3->CNT = CNT_OFFSET;
    TIM4->CNT = CNT_OFFSET;
}

/**
 * @brief 現在の左右エンコーダカウント値を取得
 * @param cnt_l 左カウント格納先ポインタ
 * @param cnt_r 右カウント格納先ポインタ
 */
void getEncoderCnt(int16_t *cnt_l, int16_t *cnt_r)
{
    *cnt_l = enc_l_cnt;
    *cnt_r = enc_r_cnt;
}

/**
 @brief 現在の左右エンコーダ累積カウント値を取得（0～65535の正値で返す）
 * @param total_l 左累積カウント格納先ポインタ
 * @param total_r 右累積カウント格納先ポインタ
 *
 * int16_t型のままでは-32768～32767でオーバーフローするため、
 * 内部でuint16_tに変換し、0～65535の範囲で返す。
 */
void getEncoderTotal(int16_t *total_l, int16_t *total_r)
{
    *total_l = (int16_t)((uint16_t)enc_l_total);
    *total_r = (int16_t)((uint16_t)enc_r_total);
}

/**
 * @brief 総走行距離を取得
 */
float getTotalDistance()
{
    return total_distance;
}

/**
 * @brief ゴール判定用距離を取得
 */
float getGoalJudgeDistance()
{
    return goal_judge_distance;
}

/**
 * @brief サイドライン判定用距離を取得
 */
float getSideLineJudgeDistance()
{
    return side_line_judge_distance;
}

/**
 * @brief 総走行距離をセット
 */
void setTotalDistance(float distance)
{
    total_distance = distance;
}

float getCrossLineIgnoreDistance(void)
{
    return distance_cross_line_ignore;
}

float getSideLineIgnoreDistance(void)
{
    return distance_side_line_ignore;
}

void clearTotalDistance()
{
    total_distance = 0;
}

void clearGoalJudgeDistance()
{
    goal_judge_distance = 0;
}

void clearSideLineJudgeDistance()
{
    side_line_judge_distance = 0;
}

void clearCrossLineIgnoreDistance(void)
{
    distance_cross_line_ignore = 0;
}

void clearSideLineIgnoreDistance(void)
{
    distance_side_line_ignore = 0;
}

void resetEncoderCnt(void)
{
    TIM3->CNT = CNT_OFFSET;
    TIM4->CNT = CNT_OFFSET;
}

float getDistance10mm(void)
{
    return distance_10mm;
}

void clearDistance10mm(void)
{
    distance_10mm = 0;
}

float getspeedcount(void)
{
    if (sab_distance_10mm >= 10)
    {
        speed_cnt += 0.1;
        sab_distance_10mm = 0;
    }
    return speed_cnt;
}

void clearspeedcount(void)
{
    speed_cnt = 0.1;
}

void getWheelDistance(float *dist_l, float *dist_r)
{
    *dist_l = (float)enc_l_total * DISTANCE_PER_CNT;
    *dist_r = (float)enc_r_total * DISTANCE_PER_CNT;
}
