#include "thruster_control.h"
#include <cmath>     // For std::abs
#include <algorithm> // For std::max, std::min
#include <stdio.h>   // For printf
#include "config.h"  // グローバル設定オブジェクト g_config を使用するため

// 現在のPWM値を保持する静的変数（実際に出力される値）
static float current_pwm_values[NUM_THRUSTERS]; // 初期化は thruster_init で行う

// 平滑化用の目標値追跡変数
static float target_pwm_values[NUM_THRUSTERS];

// 加速度制限用の前回値
static float previous_target_values[NUM_THRUSTERS];

// 姿勢固定用の変数
static bool position_hold_mode = false;
static float yaw_reference = 0.0f;
static float pitch_reference = 0.0f;
static float roll_reference = 0.0f;

// 積分制御用の変数
static float yaw_integral = 0.0f;
static float pitch_integral = 0.0f;
static float roll_integral = 0.0f;

// --- 定数設定 ---
const float MAX_ACCELERATION = 200.0f;     // PWM/秒の最大加速度
const float SMOOTH_DECELERATION = 0.3f;    // 減速時の平滑化係数
const float SMOOTH_ACCELERATION = 0.05f;   // 加速時の平滑化係数（よりゆっくり）
const float YAW_HOLD_THRESHOLD = 5.0f;     // 姿勢固定開始閾値（度/秒）
const float GYRO_DEADZONE = 1.0f;          // ジャイロのデッドゾーン（度/秒）
const float INTEGRAL_LIMIT = 500.0f;       // 積分制御の上限

// --- ヘルパー関数 ---

// 線形補正関数
static float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min)
    {
        return out_min;
    }
    x = std::max(in_min, std::min(x, in_max));
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 改良された平滑化関数（加速度制限付き）
static float advanced_smooth_interpolate(float current_value, float target_value, float dt)
{
    float diff = target_value - current_value;
    float max_change = MAX_ACCELERATION * dt;
    
    // 加速度を制限
    if (std::abs(diff) > max_change)
    {
        diff = (diff > 0) ? max_change : -max_change;
    }
    
    // 加速時と減速時で異なる平滑化係数を使用
    float smoothing_factor;
    if (target_value > current_value)
    {
        // 加速時：よりゆっくりと
        smoothing_factor = SMOOTH_ACCELERATION;
    }
    else
    {
        // 減速時：即座に応答
        smoothing_factor = SMOOTH_DECELERATION;
    }
    
    return current_value + diff * smoothing_factor;
}

// PWM値を設定するヘルパー
static void set_thruster_pwm(int channel, int pulse_width_us)
{
    int clamped_pwm = std::max(g_config.pwm_min, std::min(pulse_width_us, g_config.pwm_boost_max));
    float duty_cycle = static_cast<float>(clamped_pwm) / (1000000.0f / g_config.pwm_frequency);
    set_pwm_channel_duty_cycle(channel, duty_cycle);
}

// PID制御関数
static float pid_control(float error, float &integral, float kp, float ki, float kd, float dt, float prev_error)
{
    // 比例制御
    float proportional = kp * error;
    
    // 積分制御（ワインドアップ防止）
    integral += error * dt;
    integral = std::max(-INTEGRAL_LIMIT, std::min(integral, INTEGRAL_LIMIT));
    float integral_term = ki * integral;
    
    // 微分制御
    float derivative = kd * (error - prev_error) / dt;
    
    return proportional + integral_term + derivative;
}

// --- モジュール関数 ---

bool thruster_init()
{
    printf("Enabling PWM\n");
    set_pwm_enable(true);
    printf("Setting PWM frequency to %.1f Hz\n", g_config.pwm_frequency);
    set_pwm_freq_hz(g_config.pwm_frequency);

    // すべてのスラスターをニュートラルに初期化
    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        set_thruster_pwm(i, g_config.pwm_min);
        current_pwm_values[i] = static_cast<float>(g_config.pwm_min);
        target_pwm_values[i] = static_cast<float>(g_config.pwm_min);
        previous_target_values[i] = static_cast<float>(g_config.pwm_min);
    }
    
    // LEDチャンネルを初期状態に設定
    set_thruster_pwm(g_config.led_pwm_channel, g_config.led_pwm_off);
    
    // 姿勢固定モードを無効化
    position_hold_mode = false;
    yaw_integral = 0.0f;
    pitch_integral = 0.0f;
    roll_integral = 0.0f;
    
    printf("Thrusters initialized to PWM %d. LED on Ch%d initialized to PWM %d (OFF).\n", 
           g_config.pwm_min, g_config.led_pwm_channel, g_config.led_pwm_off);
    return true;
}

void thruster_disable()
{
    printf("Disabling PWM\n");
    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        set_thruster_pwm(i, g_config.pwm_min);
        current_pwm_values[i] = static_cast<float>(g_config.pwm_min);
        target_pwm_values[i] = static_cast<float>(g_config.pwm_min);
        previous_target_values[i] = static_cast<float>(g_config.pwm_min);
    }
    set_thruster_pwm(g_config.led_pwm_channel, g_config.led_pwm_off);
    position_hold_mode = false;
    set_pwm_enable(false);
}

// 姿勢固定モードの制御
static void update_position_hold(const AxisData &gyro_data, int pwm_corrections[4])
{
    static float prev_yaw_error = 0.0f;
    static float prev_pitch_error = 0.0f;
    static float prev_roll_error = 0.0f;
    
    const float dt = 0.02f; // 50Hz制御周期を想定
    
    // ヨー軸制御（Z軸）
    float yaw_error = -(gyro_data.z - yaw_reference);
    if (std::abs(yaw_error) > GYRO_DEADZONE)
    {
        float yaw_correction = pid_control(yaw_error, yaw_integral, 
                                          g_config.kp_yaw * 2.0f, 0.5f, 0.1f, 
                                          dt, prev_yaw_error);
        
        // ヨー補正をチャンネルに配分
        int yaw_pwm = static_cast<int>(yaw_correction);
        yaw_pwm = std::max(-300, std::min(300, yaw_pwm));
        
        if (yaw_pwm > 0)
        {
            pwm_corrections[0] += yaw_pwm;
            pwm_corrections[3] += yaw_pwm;
        }
        else
        {
            pwm_corrections[1] += std::abs(yaw_pwm);
            pwm_corrections[2] += std::abs(yaw_pwm);
        }
    }
    
    // ピッチ軸制御（Y軸）
    float pitch_error = -(gyro_data.y - pitch_reference);
    if (std::abs(pitch_error) > GYRO_DEADZONE)
    {
        float pitch_correction = pid_control(pitch_error, pitch_integral,
                                           g_config.kp_yaw * 1.5f, 0.3f, 0.05f,
                                           dt, prev_pitch_error);
        
        // ピッチ補正（前後バランス）
        int pitch_pwm = static_cast<int>(pitch_correction);
        pitch_pwm = std::max(-200, std::min(200, pitch_pwm));
        
        if (pitch_pwm > 0)
        {
            pwm_corrections[0] += pitch_pwm;
            pwm_corrections[1] += pitch_pwm;
        }
        else
        {
            pwm_corrections[2] += std::abs(pitch_pwm);
            pwm_corrections[3] += std::abs(pitch_pwm);
        }
    }
    
    // ロール軸制御（X軸）
    float roll_error = -(gyro_data.x - roll_reference);
    if (std::abs(roll_error) > GYRO_DEADZONE)
    {
        float roll_correction = pid_control(roll_error, roll_integral,
                                          g_config.kp_roll * 1.5f, 0.3f, 0.05f,
                                          dt, prev_roll_error);
        
        // ロール補正（左右バランス）
        int roll_pwm = static_cast<int>(roll_correction);
        roll_pwm = std::max(-200, std::min(200, roll_pwm));
        
        if (roll_pwm > 0)
        {
            pwm_corrections[0] += roll_pwm;
            pwm_corrections[2] += roll_pwm;
        }
        else
        {
            pwm_corrections[1] += std::abs(roll_pwm);
            pwm_corrections[3] += std::abs(roll_pwm);
        }
    }
    
    prev_yaw_error = yaw_error;
    prev_pitch_error = pitch_error;
    prev_roll_error = roll_error;
}

// 水平スラスター制御ロジック（改良版）
static void update_horizontal_thrusters(const GamepadData &data, const AxisData &gyro_data, int target_pwm_out[4])
{
    // 初期化
    for (int i = 0; i < 4; ++i)
    {
        target_pwm_out[i] = g_config.pwm_min;
    }

    bool lx_active = std::abs(data.leftThumbX) > g_config.joystick_deadzone;
    bool ly_active = std::abs(data.leftThumbY) > g_config.joystick_deadzone;
    bool rx_active = std::abs(data.rightThumbX) > g_config.joystick_deadzone;
    bool ry_active = std::abs(data.rightThumbY) > g_config.joystick_deadzone;

    // 姿勢固定モードの判定とリセット
    if (!lx_active && !ly_active && !rx_active && !ry_active)
    {
        if (!position_hold_mode)
        {
            // 姿勢固定モード開始
            position_hold_mode = true;
            yaw_reference = 0.0f;
            pitch_reference = 0.0f;
            roll_reference = 0.0f;
            yaw_integral = 0.0f;
            pitch_integral = 0.0f;
            roll_integral = 0.0f;
            printf("Position hold mode ENABLED\n");
        }
        
        // 姿勢固定制御
        int hold_corrections[4] = {0, 0, 0, 0};
        update_position_hold(gyro_data, hold_corrections);
        
        for (int i = 0; i < 4; ++i)
        {
            target_pwm_out[i] = g_config.pwm_min + hold_corrections[i];
            target_pwm_out[i] = std::max(g_config.pwm_min, 
                                       std::min(target_pwm_out[i], g_config.pwm_boost_max));
        }
        return;
    }
    else
    {
        if (position_hold_mode)
        {
            printf("Position hold mode DISABLED\n");
            position_hold_mode = false;
        }
    }

    // 通常の操作制御
    int pwm_lx[4] = {g_config.pwm_min, g_config.pwm_min, g_config.pwm_min, g_config.pwm_min};
    int pwm_ly[4] = {g_config.pwm_min, g_config.pwm_min, g_config.pwm_min, g_config.pwm_min};
    int pwm_rx[4] = {g_config.pwm_min, g_config.pwm_min, g_config.pwm_min, g_config.pwm_min};

    // 左スティックX（回転）
    if (data.leftThumbX < -g_config.joystick_deadzone)
    {
        int val = static_cast<int>(map_value(data.leftThumbX, -32768, -g_config.joystick_deadzone, 
                                           g_config.pwm_normal_max, g_config.pwm_min));
        pwm_lx[1] = val; // 前右
        pwm_lx[2] = val; // 後左
    }
    else if (data.leftThumbX > g_config.joystick_deadzone)
    {
        int val = static_cast<int>(map_value(data.leftThumbX, g_config.joystick_deadzone, 32767, 
                                           g_config.pwm_min, g_config.pwm_normal_max));
        pwm_lx[0] = val; // 前左
        pwm_lx[3] = val; // 後右
    }

    // 左スティックY（前後移動）
    if (data.leftThumbY < -g_config.joystick_deadzone)
    {
        int val = static_cast<int>(map_value(data.leftThumbY, -32768, -g_config.joystick_deadzone, 
                                           g_config.pwm_normal_max, g_config.pwm_min));
        pwm_ly[0] = val; // 前左
        pwm_ly[1] = val; // 前右
    }
    else if (data.leftThumbY > g_config.joystick_deadzone)
    {
        int val = static_cast<int>(map_value(data.leftThumbY, g_config.joystick_deadzone, 32767, 
                                           g_config.pwm_min, g_config.pwm_normal_max));
        pwm_ly[2] = val; // 後左
        pwm_ly[3] = val; // 後右
    }

    // 右スティックX（平行移動）
    if (data.rightThumbX < -g_config.joystick_deadzone)
    {
        int val = static_cast<int>(map_value(data.rightThumbX, -32768, -g_config.joystick_deadzone, 
                                           g_config.pwm_normal_max, g_config.pwm_min));
        pwm_rx[1] = val; // 前右
        pwm_rx[3] = val; // 後右
    }
    else if (data.rightThumbX > g_config.joystick_deadzone)
    {
        int val = static_cast<int>(map_value(data.rightThumbX, g_config.joystick_deadzone, 32767, 
                                           g_config.pwm_min, g_config.pwm_normal_max));
        pwm_rx[0] = val; // 前左
        pwm_rx[2] = val; // 後左
    }

    // 基本的な組み合わせ
    for (int i = 0; i < 4; ++i)
    {
        target_pwm_out[i] = std::max({pwm_lx[i], pwm_ly[i], pwm_rx[i]});
    }

    // 強化されたジャイロ補正（常時適用）
    if (rx_active || ry_active)
    {
        // ロール補正
        float roll_rate = gyro_data.x;
        int roll_correction = static_cast<int>(roll_rate * g_config.kp_roll * 2.0f);
        roll_correction = std::max(-150, std::min(150, roll_correction));
        
        target_pwm_out[0] -= roll_correction;
        target_pwm_out[1] += roll_correction;
        target_pwm_out[2] += roll_correction;
        target_pwm_out[3] -= roll_correction;

        // ピッチ補正
        float pitch_rate = gyro_data.y;
        int pitch_correction = static_cast<int>(pitch_rate * g_config.kp_yaw * 1.5f);
        pitch_correction = std::max(-150, std::min(150, pitch_correction));
        
        target_pwm_out[0] -= pitch_correction;
        target_pwm_out[1] -= pitch_correction;
        target_pwm_out[2] += pitch_correction;
        target_pwm_out[3] += pitch_correction;
    }

    // 自動ヨー補正（ラダー操作時以外）
    if (!lx_active && std::abs(gyro_data.z) > YAW_HOLD_THRESHOLD)
    {
        float yaw_rate = -gyro_data.z;
        int yaw_correction = static_cast<int>(yaw_rate * g_config.yaw_gain * 2.0f);
        yaw_correction = std::max(-300, std::min(300, yaw_correction));

        if (yaw_correction > 0)
        {
            target_pwm_out[0] = std::min(g_config.pwm_boost_max, target_pwm_out[0] + yaw_correction);
            target_pwm_out[3] = std::min(g_config.pwm_boost_max, target_pwm_out[3] + yaw_correction);
        }
        else
        {
            target_pwm_out[1] = std::min(g_config.pwm_boost_max, target_pwm_out[1] + std::abs(yaw_correction));
            target_pwm_out[2] = std::min(g_config.pwm_boost_max, target_pwm_out[2] + std::abs(yaw_correction));
        }
    }
}

// 前進/後退スラスター制御ロジック（改良版）
static int calculate_forward_reverse_pwm(int value)
{
    if (std::abs(value) <= g_config.joystick_deadzone)
    {
        return g_config.pwm_min;
    }
    
    // より滑らかな応答カーブ
    float normalized = static_cast<float>(value) / 32767.0f;
    float curved = normalized * normalized * (normalized >= 0 ? 1.0f : -1.0f); // 2次カーブ
    
    int pulse_width = static_cast<int>(map_value(curved * 32767.0f, 
                                               -32767, 32767, 
                                               g_config.pwm_min, g_config.pwm_boost_max));
    
    return std::max(g_config.pwm_min, std::min(pulse_width, g_config.pwm_boost_max));
}

// メインの更新関数（改良版）
void thruster_update(const GamepadData &gamepad_data, const AxisData &gyro_data)
{
    const float dt = 0.02f; // 50Hz制御周期
    
    // 目標PWM値の計算
    int target_horizontal_pwm[4];
    update_horizontal_thrusters(gamepad_data, gyro_data, target_horizontal_pwm);
    
    int target_forward_pwm = calculate_forward_reverse_pwm(gamepad_data.rightThumbY);

    // 改良された平滑化処理
    for (int i = 0; i < 4; ++i)
    {
        target_pwm_values[i] = static_cast<float>(target_horizontal_pwm[i]);
        current_pwm_values[i] = advanced_smooth_interpolate(
            current_pwm_values[i], 
            target_pwm_values[i], 
            dt
        );
    }
    
    // 前進/後退スラスター（より積極的な平滑化）
    float target_forward_val = static_cast<float>(target_forward_pwm);
    target_pwm_values[4] = target_forward_val;
    target_pwm_values[5] = target_forward_val;
    
    current_pwm_values[4] = advanced_smooth_interpolate(
        current_pwm_values[4], 
        target_forward_val, 
        dt
    );
    current_pwm_values[5] = advanced_smooth_interpolate(
        current_pwm_values[5], 
        target_forward_val, 
        dt
    );

    // PWM信号をスラスターに送信
    printf("--- Thruster Control (Advanced Smooth) ---\n");
    
    for (int i = 0; i < 4; ++i)
    {
        int smoothed_pwm = static_cast<int>(current_pwm_values[i]);
        smoothed_pwm = std::max(g_config.pwm_min, std::min(smoothed_pwm, g_config.pwm_boost_max));
        set_thruster_pwm(i, smoothed_pwm);
        printf("Ch%d: Target=%d, Smoothed=%d\n", i, target_horizontal_pwm[i], smoothed_pwm);
    }
    
    int smoothed_forward_pwm = static_cast<int>(current_pwm_values[4]);
    smoothed_forward_pwm = std::max(g_config.pwm_min, std::min(smoothed_forward_pwm, g_config.pwm_boost_max));
    set_thruster_pwm(4, smoothed_forward_pwm);
    set_thruster_pwm(5, smoothed_forward_pwm);
    printf("Ch4&5: Target=%d, Smoothed=%d\n", target_forward_pwm, smoothed_forward_pwm);

    // LED制御
    static int current_led_pwm = g_config.led_pwm_off;
    static bool y_button_previously_pressed = false;
    
    bool y_button_currently_pressed = (gamepad_data.buttons & GamepadButton::Y);
    
    if (y_button_currently_pressed && !y_button_previously_pressed)
    {
        current_led_pwm = (current_led_pwm == g_config.led_pwm_off) ? 
                         g_config.led_pwm_on : g_config.led_pwm_off;
    }
    y_button_previously_pressed = y_button_currently_pressed;
    
    set_thruster_pwm(g_config.led_pwm_channel, current_led_pwm);
    printf("Ch%d: LED PWM = %d (%s)\n", g_config.led_pwm_channel, current_led_pwm, 
           (current_led_pwm == g_config.led_pwm_on ? "ON" : "OFF"));
    
    if (position_hold_mode)
    {
        printf("Position Hold Mode: ACTIVE\n");
    }
    
    printf("--------------------\n");
}

// すべてのスラスターを指定されたPWM値に設定
void thruster_set_all_pwm(int pwm_value)
{
    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        set_thruster_pwm(i, pwm_value);
        current_pwm_values[i] = static_cast<float>(pwm_value);
        target_pwm_values[i] = static_cast<float>(pwm_value);
        previous_target_values[i] = static_cast<float>(pwm_value);
    }
    set_thruster_pwm(g_config.led_pwm_channel, g_config.led_pwm_off);
    position_hold_mode = false;
}