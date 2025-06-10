#ifndef THRUSTER_CONTROL_H // インクルードガード: ヘッダーファイルが複数回インクルードされるのを防ぐ
#define THRUSTER_CONTROL_H // インクルードガード

#include "gamepad.h"  // GamepadData 構造体の定義が必要なためインクルード
#include "bindings.h" // AxisData 構造体を使用するため (read_gyro() の戻り値型)

// --- 定数定義 ---
#define PWM_MIN 1100                               // PWMパルス幅の最小値 (マイクロ秒) - 後退最大または停止に対応
#define PWM_NEUTRAL 1500                           // PWMパルス幅のニュートラル値 (マイクロ秒) - 停止に対応 (ESCで一般的)
#define PWM_NORMAL_MAX 1500                        // 通常操作時の最大PWMパルス幅 (マイクロ秒) - 要確認: 現在ニュートラルと同じ
#define PWM_BOOST_MAX 1900                         // ブースト操作時の最大PWMパルス幅 (マイクロ秒) - 前進最大に対応
#define PWM_FREQUENCY 50.0f                        // PWM信号の周波数 (Hz)
#define PWM_PERIOD_US (1000000.0f / PWM_FREQUENCY) // PWM信号の周期 (マイクロ秒) - 50Hzの場合20000us

#define JOYSTICK_DEADZONE 6500 // ジョイスティック入力のデッドゾーン閾値 (この値以下は無視)
#define NUM_THRUSTERS 6        // 制御対象のスラスター総数 (Ch0-3 水平, Ch4-5 前進/後退)

// --- LED制御用定数 ---
#define LED_PWM_CHANNEL 9      // LEDを制御するPWMチャンネル番号
#define LED_PWM_ON 1900        // LED点灯時のPWM値
#define LED_PWM_OFF 1100       // LED消灯時のPWM値 (1500以下で消灯との指示に基づき1500に設定)

// --- 関数のプロトタイプ宣言 ---
// スラスター制御モジュールを初期化する (PWM設定など)
bool thruster_init();
// スラスター制御を無効化する (PWM停止など)
void thruster_disable();
// ゲームパッドデータとジャイロデータに基づいてすべてのスラスターのPWM出力を更新する
void thruster_update(const GamepadData &gamepad_data, const AxisData &gyro_data);
// 全てのスラスターを指定されたPWM値に設定し、LEDをオフにする (フェイルセーフ用)
void thruster_set_all_pwm(int pwm_value);
// ヘルパー関数（他の場所で必要ない場合は .cpp 内部に保持できます）
// float map_value(float x, float in_min, float in_max, float out_min, float out_max);

#endif // THRUSTER_CONTROL_H
