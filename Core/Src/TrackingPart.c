#include "TrackingPart.h"

#define DELTA_T 0.001

static int8_t trace_flag;
static uint8_t i_clear_flag;
float tracking_term; // static修飾子を削除
static bool Unable_to_run_flag;
static float mo_l_Deb;
static float mo_r_Deb;
static float pre_diff;
float mon_velo_term;
static float baseSpeed = 0; // baseSpeedを初期化

// mainのlion変数をextern宣言
extern int lion;

void ControlLineTracking(void)
{
    float p, d;
    static float i;
    float kp = 1.8325;
    float kd = 0.0638;
    float diff = 0;
    if (trace_flag == 1)
    {
        if (i_clear_flag == 1)
        {
            i = 0;
            i_clear_flag = 0;
        }
        diff = ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) / 5) - ((sensor[6] + sensor[7] + sensor[8] + sensor[9] + sensor[10] + sensor[11]) / 5);
        p = kp * diff;
        d = kd * (diff - pre_diff) / DELTA_T;
        tracking_term = p + d + i;

        pre_diff = diff;
    }
}

void TraceFlip(void)
{
    if (trace_flag == 1)
    {
        float velo_ctrl_term = getVelocityControlTerm();
        float limit = MAX_COUNTER_PERIOD * 0.8;

        if (velo_ctrl_term >= limit)
            velo_ctrl_term = limit;
        else if (velo_ctrl_term <= -limit)
            velo_ctrl_term = -limit;

        float exceeded = 0;
        if (velo_ctrl_term + tracking_term >= MAX_COUNTER_PERIOD)
        {
            exceeded = (velo_ctrl_term + tracking_term) - MAX_COUNTER_PERIOD;
        }
        else if (velo_ctrl_term - tracking_term <= -MAX_COUNTER_PERIOD)
        {
            exceeded = -MAX_COUNTER_PERIOD - (velo_ctrl_term - tracking_term);
        }
        velo_ctrl_term -= exceeded;
        tracking_term += exceeded;

        float motor_left = baseSpeed + velo_ctrl_term + tracking_term; // baseSpeedを追加
        float motor_right = baseSpeed + velo_ctrl_term - tracking_term; // baseSpeedを追加

        mon_velo_term = velo_ctrl_term;

        setMotor(motor_left, motor_right);
    }
    else
    {
        setMotor(0, 0);
    }
}

void startTracking(void)
{
    trace_flag = 1;
    i_clear_flag = 1;
}

void stopTracking(void)
{
    trace_flag = 0;
    tracking_term = 0;
}

void CourseOut(void)
{
    uint16_t all_sensor;
    static uint16_t unable_cnt;
    all_sensor = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] +
                  sensor[6] + sensor[7] + sensor[8] + sensor[9] + sensor[10] + sensor[11]) /
                 12;
    if (all_sensor > 850)
    {
        unable_cnt++;
    }
    else
        unable_cnt = 0;

    if (unable_cnt >= COURSEOUT)
    {
        Unable_to_run_flag = true;
        lion = 7; // mainにあるlionと同じ変数に戻す
        setMotor(0, 0);
    }
    else
    {
        Unable_to_run_flag = false;
    }
}

void debugmotor(float mon_deb_l, float mon_deb_r)
{
    mo_l_Deb = mon_deb_l;
    mo_r_Deb = mon_deb_r;
}

bool getUnableToRunFlag(void)
{
    return Unable_to_run_flag;
}

void setBaseSpeed(float speed) {
    baseSpeed = speed; // baseSpeedを設定する関数
}

float getBaseSpeed(void) {
    return baseSpeed; // baseSpeedを取得する関数
}
