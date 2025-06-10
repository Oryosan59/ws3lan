#include "thruster_control.h"
#include <cmath>     // For std::abs
#include <algorithm> // For std::max, std::min
#include <stdio.h>   // For printf

// --- ヘルパー関数 ---

// 線形補正関数
static float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min)
    {
        // ゼロ除算を回避
        return out_min;
    }
    // マッピング前に入力値を指定範囲内にクランプ
    x = std::max(in_min, std::min(x, in_max));
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// PWM値を設定するヘルパー (範囲チェックとデューティサイクル計算を含む)
static void set_thruster_pwm(int channel, int pulse_width_us)
{
    // PWM値が有効な動作範囲内にあることを保証するためにクランプ
    // 注意: クランプの上限として PWM_BOOST_MAX を使用
    int clamped_pwm = std::max(PWM_MIN, std::min(pulse_width_us, PWM_BOOST_MAX));

    // デューティサイクルを計算
    float duty_cycle = static_cast<float>(clamped_pwm) / PWM_PERIOD_US;

    // 指定されたチャンネルのPWMデューティサイクルを設定
    set_pwm_channel_duty_cycle(channel, duty_cycle);

    // デバッグ出力 (オプション)
    // printf("Ch%d: Set PWM = %d (Clamped: %d), Duty = %.4f\n", channel, pulse_width_us, clamped_pwm, duty_cycle);
}

// --- モジュール関数 ---

bool thruster_init()
{
    printf("Enabling PWM\n");
    set_pwm_enable(true);
    printf("Setting PWM frequency to %.1f Hz\n", PWM_FREQUENCY);
    set_pwm_freq_hz(PWM_FREQUENCY);
    // すべてのスラスターをニュートラル/最小値に初期化？
    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        set_thruster_pwm(i, PWM_MIN); // または適用可能であれば PWM_NEUTRAL
    }
    // LEDチャンネルを初期状態 (OFF) に設定
    set_thruster_pwm(LED_PWM_CHANNEL, LED_PWM_OFF);
    printf("Thrusters initialized to PWM %d. LED on Ch%d initialized to PWM %d (OFF).\n", PWM_MIN, LED_PWM_CHANNEL, LED_PWM_OFF);
    return true; // 初期化関数が簡単にステータスを返さないと仮定
}

void thruster_disable()
{
    printf("Disabling PWM\n");
    // 無効にする前に、オプションですべてのスラスターをニュートラル/最小値に設定
    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        set_thruster_pwm(i, PWM_MIN); // または PWM_NEUTRAL
    }
    // LEDチャンネルをOFFに設定
    set_thruster_pwm(LED_PWM_CHANNEL, LED_PWM_OFF);
    set_pwm_enable(false);
}

// 水平スラスター制御ロジック (updateThrustersFromSticksの内容を移植・調整)
static void update_horizontal_thrusters(const GamepadData &data, const AxisData &gyro_data, int pwm_out[4])
{
    // Initialize output PWM array to neutral/min
    for (int i = 0; i < 4; ++i) // 出力PWM配列をニュートラル/最小値に初期化
        pwm_out[i] = PWM_MIN;

    bool lx_active = std::abs(data.leftThumbX) > JOYSTICK_DEADZONE;
    bool rx_active = std::abs(data.rightThumbX) > JOYSTICK_DEADZONE;

    int pwm_lx[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
    int pwm_rx[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};

    // Lx (回転) の寄与 (PWM_MIN - PWM_NORMAL_MAX にマッピング)
    if (data.leftThumbX < -JOYSTICK_DEADZONE)
    { // 左旋回
        int val = static_cast<int>(map_value(data.leftThumbX, -32768, -JOYSTICK_DEADZONE, PWM_NORMAL_MAX, PWM_MIN));
        pwm_lx[1] = val; // Ch 1 (前右)
        pwm_lx[2] = val; // Ch 2 (後左)
    }
    else if (data.leftThumbX > JOYSTICK_DEADZONE)
    { // 右旋回
        int val = static_cast<int>(map_value(data.leftThumbX, JOYSTICK_DEADZONE, 32767, PWM_MIN, PWM_NORMAL_MAX));
        pwm_lx[0] = val; // Ch 0 (前左)
        pwm_lx[3] = val; // Ch 3 (後右)
    }

    // Rx (平行移動) の寄与 (PWM_MIN - PWM_NORMAL_MAX にマッピング)
    if (data.rightThumbX < -JOYSTICK_DEADZONE)
    { // 左平行移動
        int val = static_cast<int>(map_value(data.rightThumbX, -32768, -JOYSTICK_DEADZONE, PWM_NORMAL_MAX, PWM_MIN));
        pwm_rx[1] = val; // Ch 1 (前右) - FR/RLが左に押すX構成と仮定
        pwm_rx[3] = val; // Ch 3 (後右)  - FR/RLが左に押すX構成と仮定
    }
    else if (data.rightThumbX > JOYSTICK_DEADZONE)
    { // 右平行移動
        int val = static_cast<int>(map_value(data.rightThumbX, JOYSTICK_DEADZONE, 32767, PWM_MIN, PWM_NORMAL_MAX));
        pwm_rx[0] = val; // Ch 0 (前左) - FL/RRが右に押すX構成と仮定
        pwm_rx[2] = val; // Ch 2 (後左)  - FL/RRが右に押すX構成と仮定
    }

    // 両方のスティックがアクティブな場合、寄与を結合してブーストを適用
    if (lx_active && rx_active)
    {
        const int boost_range = PWM_BOOST_MAX - PWM_NORMAL_MAX;
        int abs_lx = std::abs(data.leftThumbX);
        int abs_rx = std::abs(data.rightThumbX);
        int weaker_input_abs = std::min(abs_lx, abs_rx);
        int boost_add = static_cast<int>(map_value(weaker_input_abs, JOYSTICK_DEADZONE, 32768, 0, boost_range));

        // スティックの方向に基づいてブーストされるチャンネルを決定
        if (data.leftThumbX < 0 && data.rightThumbX < 0)
        { // 左旋回 + 左平行移動 -> Ch 1 (FR) をブースト
            pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]);
            pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]) + boost_add;
            pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]);
            pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]);
        }
        else if (data.leftThumbX < 0 && data.rightThumbX > 0)
        { // 左旋回 + 右平行移動 -> Ch 2 (RL) をブースト
            pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]);
            pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]);
            pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]) + boost_add;
            pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]);
        }
        else if (data.leftThumbX > 0 && data.rightThumbX < 0)
        { // 右旋回 + 左平行移動 -> Ch 3 (RR) をブースト
            pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]);
            pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]);
            pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]);
            pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]) + boost_add;
        }
        else
        { // 右旋回 + 右平行移動 -> Ch 0 (FL) をブースト
            pwm_out[0] = std::max(pwm_lx[0], pwm_rx[0]) + boost_add;
            pwm_out[1] = std::max(pwm_lx[1], pwm_rx[1]);
            pwm_out[2] = std::max(pwm_lx[2], pwm_rx[2]);
            pwm_out[3] = std::max(pwm_lx[3], pwm_rx[3]);
        }
    }
    else
    {
        // 一方のスティックのみアクティブ、またはどちらも非アクティブ: 単純な組み合わせ (最大値)
        for (int i = 0; i < 4; ++i)
        {
            pwm_out[i] = std::max(pwm_lx[i], pwm_rx[i]);
        }
    }

    // --- ジャイロによるロール安定化補正 (エルロン操作時) ---
    // rx_active は data.rightThumbX (エルロン操作) がデッドゾーン外であるかを示すフラグ
    if (rx_active) // エルロン操作中のみ安定化制御を行う
    {
        // --- ロール補正 ---
        // 仮定: gyro_data.x がロール軸の角速度 (右へのロールが正)
        //       単位が deg/s の場合を想定。rad/s ならKp値を調整。
        float roll_rate = gyro_data.x;

        // P制御ゲイン (要調整: この値は非常に小さい値から試してください)
        const float Kp_roll = 0.2f; // ★★★ 要調整 ★★★

        // 補正値の計算
        // roll_rate > 0 (右にロール) の場合、左回転の力を加えたい。
        // 左回転は Ch1(前右)とCh2(後左)を強く、Ch0(前左)とCh3(後右)を弱くする。
        // correction_pwm_roll が正の時に左回転を強める。
        int correction_pwm_roll = static_cast<int>(roll_rate * Kp_roll);

        // pwm_out にロール補正を適用 (回転スラスターのバランスを調整)
        // Ch0 (前左): 減らす (左回転のため)
        // Ch1 (前右): 増やす (左回転のため)
        // Ch2 (後左): 増やす (左回転のため)
        // Ch3 (後右): 減らす (左回転のため)
        pwm_out[0] -= correction_pwm_roll;
        pwm_out[1] += correction_pwm_roll;
        pwm_out[2] += correction_pwm_roll;
        pwm_out[3] -= correction_pwm_roll;

        // --- ヨー補正 (Z軸回転の調整) ---
        // 仮定: gyro_data.z がヨー軸の角速度 (右へのヨーイングが正)
        //       単位が deg/s の場合を想定。rad/s ならKp値を調整。
        //       センサーのZ軸が機体のヨー軸と一致しているか、符号が正しいか確認してください。
        float yaw_rate = gyro_data.z;

        // P制御ゲイン (ヨー用 - 要調整)
        const float Kp_yaw = 0.15f; // ★★★ 要調整 ★★★ (ロール用とは別に調整)

        // ヨー補正値の計算
        // yaw_rate > 0 (右にヨー) の場合、左ヨーの力を加えたい。
        // 左ヨーは Ch1(前右)とCh2(後左)を強く、Ch0(前左)とCh3(後右)を弱くする (回転制御と同じ)。
        // correction_pwm_yaw が正の時に左ヨーを強める。
        int correction_pwm_yaw = static_cast<int>(yaw_rate * Kp_yaw);

        // pwm_out にヨー補正を適用 (回転スラスターのバランスを調整)
        pwm_out[0] -= correction_pwm_yaw; // 左ヨーを助ける (Ch0を弱める)
        pwm_out[1] += correction_pwm_yaw; // 左ヨーを助ける (Ch1を強める)
        pwm_out[2] += correction_pwm_yaw; // 左ヨーを助ける (Ch2を強める)
        pwm_out[3] -= correction_pwm_yaw; // 左ヨーを助ける (Ch3を弱める)
    }

    // --- GyroによるYaw補正 (Rx入力時にZ軸回転しないよう補正) ---
    if (!lx_active)
    {
        // GyroのZ軸角速度が±一定以上なら補正を行う
        const float yaw_threshold_dps = 2.0f; // deg/s単位のしきい値（調整可能）
        const float yaw_gain = 50.0f;         // 補正のゲイン（調整可能）

        float yaw_rate = -gyro_data.z; // Z軸の角速度[deg/s]

        if (std::abs(yaw_rate) > yaw_threshold_dps)
        {
            // Yaw補正のLx寄与を生成（符号反転：回転を打ち消す）
            int yaw_pwm = static_cast<int>(yaw_rate * -yaw_gain);

            // yaw_pwmを安全な範囲にクリップ（±で出る値を考慮してオフセット加算）
            yaw_pwm = std::max(-400, std::min(400, yaw_pwm));

            // 既存のpwm_rxにLx補正を加える
            // Lxと同様のチャネルへ適用（Lxと同じ方向に推力を加えることで補正）
            if (yaw_pwm < 0)
            {
                // 左旋回を打ち消す（＝右回転） → Ch 0,3を加算
                pwm_out[0] = std::min(PWM_BOOST_MAX, pwm_out[0] + std::abs(yaw_pwm));
                pwm_out[3] = std::min(PWM_BOOST_MAX, pwm_out[3] + std::abs(yaw_pwm));
            }
            else
            {
                // 右旋回を打ち消す（＝左回転） → Ch 1,2を加算
                pwm_out[1] = std::min(PWM_BOOST_MAX, pwm_out[1] + yaw_pwm);
                pwm_out[2] = std::min(PWM_BOOST_MAX, pwm_out[2] + yaw_pwm);
            }
        }
    }

} // 最終的なクランプは set_thruster_pwm で行われる

// 前進/後退スラスター制御ロジック
static int calculate_forward_reverse_pwm(int value)
{
    int pulse_width;
    // 定数を直接使用 (ヘッダーファイルで定義されている値)
    const int current_max_pwm = PWM_BOOST_MAX; // 1900
    const int current_min_pwm = PWM_MIN;       // 1100

    // value が JOYSTICK_DEADZONE 以下 (後退方向またはデッドゾーン内) の場合
    if (value <= JOYSTICK_DEADZONE)
    {
        // PWMを PWM_MIN (1100) に固定
        pulse_width = PWM_MIN;
    }
    else
    { // value > JOYSTICK_DEADZONE (前進)
        // 前進推力: 入力 JOYSTICK_DEADZONE ~ 32767 を 出力 PWM_MIN ~ PWM_BOOST_MAX にマッピング
        pulse_width = static_cast<int>(map_value(value, JOYSTICK_DEADZONE, 32767, PWM_MIN, current_max_pwm));
    }
    // 最終的なクランプ処理は set_thruster_pwm 関数内で行われます
    return pulse_width;
}

// メインの更新関数
void thruster_update(const GamepadData &gamepad_data, const AxisData &gyro_data)
{
    int horizontal_pwm[4];
    update_horizontal_thrusters(gamepad_data, gyro_data, horizontal_pwm);

    // 元の main() に基づき、チャンネル4が前進/後退用と仮定
    int forward_pwm = calculate_forward_reverse_pwm(gamepad_data.rightThumbY);

    // --- PWM信号をスラスターに送信 ---
    printf("--- Thruster and LED PWM ---\n");
    // 水平スラスター
    for (int i = 0; i < 4; ++i)
    {
        set_thruster_pwm(i, horizontal_pwm[i]);                // 水平スラスターPWM設定
        printf("Ch%d: Hori PWM = %d\n", i, horizontal_pwm[i]); // デバッグ
    }
    // 前進/後退スラスター
    set_thruster_pwm(4, forward_pwm);
    printf("Ch4: FwdRev PWM = %d\n", forward_pwm);
    set_thruster_pwm(5, forward_pwm);
    printf("Ch5: FwdRev PWM = %d\n", forward_pwm); // Ch5のデバッグ出力追加

    // --- LED制御 ---
    // 静的変数を導入してLEDの現在のPWM値とYボタンの前回状態を保持
    static int current_led_pwm = LED_PWM_OFF; // 初期状態は消灯
    static bool y_button_previously_pressed = false;

    // 現在のYボタンの押下状態を取得
    // gamepad.h の GamepadButton::Y の値が送信側のデータと一致している必要があります。
    // 現在の定義 Y = 0x32768 (10進数) は 0x8000 (16進数) と等価であり、
    // gamepad_data.buttons (uint16_t) とのビットAND演算で正しく動作する想定です。
    bool y_button_currently_pressed = (gamepad_data.buttons & GamepadButton::Y);

    // Yボタンが押された瞬間 (前回押されておらず、今回押された場合) にLEDの状態をトグル
    if (y_button_currently_pressed && !y_button_previously_pressed)
    {
        if (current_led_pwm == LED_PWM_OFF)
        {
            current_led_pwm = LED_PWM_ON;
        }
        else
        {
            current_led_pwm = LED_PWM_OFF;
        }
    }
    // Yボタンの現在の状態を次回のために保存
    y_button_previously_pressed = y_button_currently_pressed;

    set_thruster_pwm(LED_PWM_CHANNEL, current_led_pwm);
    printf("Ch%d: LED PWM = %d (%s)\n", LED_PWM_CHANNEL, current_led_pwm, (current_led_pwm == LED_PWM_ON ? "ON" : "OFF"));

    printf("--------------------\n");
}

// すべてのスラスターを指定されたPWM値に設定し、LEDをオフにする関数
void thruster_set_all_pwm(int pwm_value)
{
    // printf("フェイルセーフ: 全スラスターをPWM %d に設定、LEDをオフ\n", pwm_value);
    for (int i = 0; i < NUM_THRUSTERS; ++i) // NUM_THRUSTERS は Ch0 から Ch5 までを想定
    {
        // set_thruster_pwm はクランプ処理を含むので安全
        set_thruster_pwm(i, pwm_value);
    }
    // フェイルセーフ時にはLEDもオフにする
    set_thruster_pwm(LED_PWM_CHANNEL, LED_PWM_OFF);
}
