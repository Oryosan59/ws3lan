#include "thruster_control.h"
#include <cmath>     // For std::abs, std::pow
#include <algorithm> // For std::max, std::min
#include <stdio.h>   // For printf
#include "config.h"  // グローバル設定オブジェクト g_config を使用するため

// 現在のPWM値を保持する静的変数（実際に出力される値）
static float current_pwm_values[NUM_THRUSTERS]; // 初期化は thruster_init で行う

// 姿勢制御用の積分項
static float yaw_integral = 0.0f;
static float roll_integral = 0.0f;

// 前回の値を記録（微分項計算用）
static float prev_yaw_rate = 0.0f;
static float prev_roll_rate = 0.0f;

// スティック入力の変化率を記録
static float prev_stick_values[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // LX, RX, LY, RY

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

// 二次関数的な応答カーブ（加速時用）
static float quadratic_response(float input, float deadzone, float max_input)
{
    if (std::abs(input) <= deadzone) {
        return 0.0f;
    }
    
    float normalized = (std::abs(input) - deadzone) / (max_input - deadzone);
    normalized = std::max(0.0f, std::min(1.0f, normalized));
    
    // 二次関数的な応答（最初は緩やか、後半は急激）
    float response = normalized * normalized;
    
    return (input >= 0) ? response : -response;
}

// 動的平滑化関数（加速時は滑らか、減速時は即座）
static float dynamic_smooth_interpolate(float current_value, float target_value, 
                                       float accel_factor, float decel_factor)
{
    if (std::abs(target_value) > std::abs(current_value)) {
        // 加速時：滑らかな応答
        return current_value + (target_value - current_value) * accel_factor;
    } else {
        // 減速時：即座の応答
        return current_value + (target_value - current_value) * decel_factor;
    }
}

// PWM値を設定するヘルパー (範囲チェックとデューティサイクル計算を含む)
static void set_thruster_pwm(int channel, int pulse_width_us)
{
    int clamped_pwm = std::max(g_config.pwm_min, std::min(pulse_width_us, g_config.pwm_boost_max));
    float duty_cycle = static_cast<float>(clamped_pwm) / (1000000.0f / g_config.pwm_frequency);
    set_pwm_channel_duty_cycle(channel, duty_cycle);
}

// PID制御関数
static float pid_control(float error, float &integral, float &prev_error, 
                        float kp, float ki, float kd, float dt = 0.01f)
{
    integral += error * dt;
    
    // 積分項の飽和防止
    float integral_max = 100.0f;
    integral = std::max(-integral_max, std::min(integral, integral_max));
    
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    
    return kp * error + ki * integral + kd * derivative;
}

// --- モジュール関数 ---

bool thruster_init()
{
    printf("Enabling PWM\n");
    set_pwm_enable(true);
    printf("Setting PWM frequency to %.1f Hz\n", g_config.pwm_frequency);
    set_pwm_freq_hz(g_config.pwm_frequency);

    // すべてのスラスターをニュートラル/最小値に初期化
    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        set_thruster_pwm(i, g_config.pwm_min);
        current_pwm_values[i] = static_cast<float>(g_config.pwm_min);
    }
    
    // 制御用変数の初期化
    yaw_integral = 0.0f;
    roll_integral = 0.0f;
    prev_yaw_rate = 0.0f;
    prev_roll_rate = 0.0f;
    
    for (int i = 0; i < 4; ++i) {
        prev_stick_values[i] = 0.0f;
    }
    
    // LEDチャンネルを初期状態 (OFF) に設定
    set_thruster_pwm(g_config.led_pwm_channel, g_config.led_pwm_off);
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
    }
    set_thruster_pwm(g_config.led_pwm_channel, g_config.led_pwm_off);
    set_pwm_enable(false);
    
    // 制御用変数のリセット
    yaw_integral = 0.0f;
    roll_integral = 0.0f;
    prev_yaw_rate = 0.0f;
    prev_roll_rate = 0.0f;
}

// 水平スラスター制御ロジック（改良版）
static void update_horizontal_thrusters(const GamepadData &data, const AxisData &gyro_data, 
                                       int target_pwm_out[4])
{
    // Initialize target PWM array to neutral/min
    for (int i = 0; i < 4; ++i) {
        target_pwm_out[i] = g_config.pwm_min;
    }

    // スティック入力の正規化と二次関数的応答の適用
    float lx_normalized = quadratic_response(data.leftThumbX, g_config.joystick_deadzone, 32767);
    float rx_normalized = quadratic_response(data.rightThumbX, g_config.joystick_deadzone, 32767);
    
    bool lx_active = std::abs(data.leftThumbX) > g_config.joystick_deadzone;
    bool rx_active = std::abs(data.rightThumbX) > g_config.joystick_deadzone;

    int pwm_lx[4] = {g_config.pwm_min, g_config.pwm_min, g_config.pwm_min, g_config.pwm_min};
    int pwm_rx[4] = {g_config.pwm_min, g_config.pwm_min, g_config.pwm_min, g_config.pwm_min};

    // Lx (回転) の寄与
    if (lx_normalized < 0) { // 左旋回
        int val = static_cast<int>(map_value(lx_normalized, -1.0f, 0.0f, 
                                           g_config.pwm_normal_max, g_config.pwm_min));
        pwm_lx[1] = val; // Ch 1 (前右)
        pwm_lx[2] = val; // Ch 2 (後左)
    } else if (lx_normalized > 0) { // 右旋回
        int val = static_cast<int>(map_value(lx_normalized, 0.0f, 1.0f, 
                                           g_config.pwm_min, g_config.pwm_normal_max));
        pwm_lx[0] = val; // Ch 0 (前左)
        pwm_lx[3] = val; // Ch 3 (後右)
    }

    // Rx (平行移動) の寄与
    if (rx_normalized < 0) { // 左平行移動
        int val = static_cast<int>(map_value(rx_normalized, -1.0f, 0.0f, 
                                           g_config.pwm_normal_max, g_config.pwm_min));
        pwm_rx[1] = val; // Ch 1 (前右)
        pwm_rx[3] = val; // Ch 3 (後右)
    } else if (rx_normalized > 0) { // 右平行移動
        int val = static_cast<int>(map_value(rx_normalized, 0.0f, 1.0f, 
                                           g_config.pwm_min, g_config.pwm_normal_max));
        pwm_rx[0] = val; // Ch 0 (前左)
        pwm_rx[2] = val; // Ch 2 (後左)
    }

    // 複合動作時のブースト処理
    if (lx_active && rx_active) {
        const int boost_range = g_config.pwm_boost_max - g_config.pwm_normal_max;
        float combined_intensity = (std::abs(lx_normalized) + std::abs(rx_normalized)) / 2.0f;
        int boost_add = static_cast<int>(combined_intensity * boost_range);

        // スティックの方向に基づいてブーストされるチャンネルを決定
        if (lx_normalized < 0 && rx_normalized < 0) { // 左旋回 + 左平行移動
            target_pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]) + boost_add;
            target_pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]);
            target_pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]);
            target_pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]);
        } else if (lx_normalized < 0 && rx_normalized > 0) { // 左旋回 + 右平行移動
            target_pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]) + boost_add;
            target_pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]);
            target_pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]);
            target_pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]);
        } else if (lx_normalized > 0 && rx_normalized < 0) { // 右旋回 + 左平行移動
            target_pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]) + boost_add;
            target_pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]);
            target_pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]);
            target_pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]);
        } else { // 右旋回 + 右平行移動
            target_pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]) + boost_add;
            target_pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]);
            target_pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]);
            target_pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]);
        }
    } else {
        // 単一スティック操作時
        for (int i = 0; i < 4; ++i) {
            target_pwm_out[i] = std::max(pwm_lx[i], pwm_rx[i]);
        }
    }

    // --- 強化されたジャイロ姿勢制御 ---
    
    // 自動ヨー制御（ラダー操作なし時）
    if (!lx_active) {
        float yaw_rate = gyro_data.z; // Z軸はヨーレート
        
        if (std::abs(yaw_rate) > g_config.yaw_threshold_dps) {
            // PID制御でヨー安定化
            float yaw_error = -yaw_rate; // 目標は0なので誤差は-yaw_rate
            float yaw_correction = pid_control(yaw_error, yaw_integral, prev_yaw_rate, 
                                             g_config.kp_yaw * 2.0f, 0.1f, 0.05f);
            
            int yaw_pwm = static_cast<int>(yaw_correction);
            yaw_pwm = std::max(-300, std::min(300, yaw_pwm));
            
            // ヨー補正を各スラスターに適用
            if (yaw_pwm > 0) { // 右旋回補正
                target_pwm_out[0] = std::min(g_config.pwm_boost_max, target_pwm_out[0] + yaw_pwm);
                target_pwm_out[3] = std::min(g_config.pwm_boost_max, target_pwm_out[3] + yaw_pwm);
            } else { // 左旋回補正
                target_pwm_out[1] = std::min(g_config.pwm_boost_max, target_pwm_out[1] + std::abs(yaw_pwm));
                target_pwm_out[2] = std::min(g_config.pwm_boost_max, target_pwm_out[2] + std::abs(yaw_pwm));
            }
        }
    }

    // 自動ロール制御（エルロン操作時の安定化）
    if (rx_active) {
        float roll_rate = gyro_data.x; // X軸はロールレート
        float roll_error = -roll_rate; // 目標は0
        float roll_correction = pid_control(roll_error, roll_integral, prev_roll_rate,
                                          g_config.kp_roll * 3.0f, 0.05f, 0.02f);
        
        int roll_pwm = static_cast<int>(roll_correction);
        roll_pwm = std::max(-200, std::min(200, roll_pwm));
        
        // ロール補正を各スラスターに適用
        target_pwm_out[0] = std::max(g_config.pwm_min, target_pwm_out[0] - roll_pwm);
        target_pwm_out[1] = std::max(g_config.pwm_min, target_pwm_out[1] + roll_pwm);
        target_pwm_out[2] = std::max(g_config.pwm_min, target_pwm_out[2] + roll_pwm);
        target_pwm_out[3] = std::max(g_config.pwm_min, target_pwm_out[3] - roll_pwm);
    }
}

// 前進/後退スラスター制御ロジック（改良版）
static int calculate_forward_reverse_pwm(int value)
{
    if (std::abs(value) <= g_config.joystick_deadzone) {
        return g_config.pwm_min;
    }
    
    // 二次関数的応答を適用
    float normalized = quadratic_response(value, g_config.joystick_deadzone, 32767);
    int pulse_width = static_cast<int>(map_value(normalized, 0.0f, 1.0f, 
                                               g_config.pwm_min, g_config.pwm_boost_max));
    
    return pulse_width;
}

// メインの更新関数（改良された平滑化機能付き）
void thruster_update(const GamepadData &gamepad_data, const AxisData &gyro_data)
{
    // 目標PWM値の計算
    int target_horizontal_pwm[4];
    update_horizontal_thrusters(gamepad_data, gyro_data, target_horizontal_pwm);

    int target_forward_pwm = calculate_forward_reverse_pwm(gamepad_data.rightThumbY);

    // --- 動的平滑化処理 ---
    
    // 水平スラスター (Ch0-3) の動的平滑化
    for (int i = 0; i < 4; ++i) {
        current_pwm_values[i] = dynamic_smooth_interpolate(
            current_pwm_values[i], 
            static_cast<float>(target_horizontal_pwm[i]),
            g_config.smoothing_factor_horizontal * 0.3f, // 加速時はより滑らか
            0.8f  // 減速時は即座に応答
        );
    }
    
    // 前進/後退スラスター (Ch4, Ch5) の動的平滑化
    float target_forward_val = static_cast<float>(target_forward_pwm);
    
    current_pwm_values[4] = dynamic_smooth_interpolate(
        current_pwm_values[4], 
        target_forward_val,
        g_config.smoothing_factor_vertical * 0.4f, // 加速時
        0.9f  // 減速時
    );
    
    current_pwm_values[5] = dynamic_smooth_interpolate(
        current_pwm_values[5], 
        target_forward_val,
        g_config.smoothing_factor_vertical * 0.4f, // 加速時
        0.9f  // 減速時
    );

    // --- PWM信号をスラスターに送信 ---
    printf("--- Enhanced Thruster Control (Quadratic Response + Dynamic Smoothing) ---\n");
    
    // 水平スラスター
    for (int i = 0; i < 4; ++i) {
        int smoothed_pwm = static_cast<int>(current_pwm_values[i]);
        set_thruster_pwm(i, smoothed_pwm);
        printf("Ch%d: Target=%d, Smoothed=%d\n", i, target_horizontal_pwm[i], smoothed_pwm);
    }
    
    // 前進/後退スラスター
    int smoothed_forward_pwm = static_cast<int>(current_pwm_values[4]);
    set_thruster_pwm(4, smoothed_forward_pwm);
    set_thruster_pwm(5, smoothed_forward_pwm);
    printf("Ch4&5: Target=%d, Smoothed=%d\n", target_forward_pwm, smoothed_forward_pwm);

    // --- LED制御 ---
    static int current_led_pwm = g_config.led_pwm_off;
    static bool y_button_previously_pressed = false;

    bool y_button_currently_pressed = (gamepad_data.buttons & GamepadButton::Y);

    if (y_button_currently_pressed && !y_button_previously_pressed) {
        current_led_pwm = (current_led_pwm == g_config.led_pwm_off) ? 
                         g_config.led_pwm_on : g_config.led_pwm_off;
    }
    y_button_previously_pressed = y_button_currently_pressed;

    set_thruster_pwm(g_config.led_pwm_channel, current_led_pwm);
    printf("Ch%d: LED PWM = %d (%s)\n", g_config.led_pwm_channel, current_led_pwm, 
           (current_led_pwm == g_config.led_pwm_on ? "ON" : "OFF"));

    // ジャイロ情報をデバッグ出力
    printf("Gyro: Roll=%.2f°/s, Yaw=%.2f°/s\n", gyro_data.x, gyro_data.z);
    printf("--------------------\n");
}

// すべてのスラスターを指定されたPWM値に設定
void thruster_set_all_pwm(int pwm_value)
{
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        set_thruster_pwm(i, pwm_value);
        current_pwm_values[i] = static_cast<float>(pwm_value);
    }
    set_thruster_pwm(g_config.led_pwm_channel, g_config.led_pwm_off);
    
    // 制御用変数のリセット
    yaw_integral = 0.0f;
    roll_integral = 0.0f;
}