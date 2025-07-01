#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <map>
#include <mutex> // std::mutex を使用するため

// GStreamerパイプラインごとの設定を保持する構造体
struct GStreamerConfig {
    std::string device;
    int port;
    int width;
    int height;
    int framerate_num;
    int framerate_den;
    bool is_h264_native_source;
    int rtp_payload_type;
    int rtp_config_interval;
    int x264_bitrate;
    std::string x264_tune;
    std::string x264_speed_preset;
};

// アプリケーション全体の設定を保持する構造体
struct AppConfig {
    // PWM
    int pwm_min;
    int pwm_neutral;
    int pwm_normal_max;
    int pwm_boost_max;
    double pwm_frequency;

    // Joystick
    int joystick_deadzone;

    // LED
    int led_channel;
    int led_on_value;
    int led_off_value;

    // Thruster Control
    double smoothing_factor_horizontal;
    double smoothing_factor_vertical;
    double kp_roll;
    double kp_yaw;
    double yaw_threshold_dps;
    double yaw_gain;

    // Network (UDP for gamepad/sensor)
    int network_recv_port;
    int network_send_port;
    std::string client_host;
    double connection_timeout_seconds;

    // Application
    unsigned int sensor_send_interval;
    unsigned int loop_delay_us;

    // Config Synchronizer (TCP)
    std::string wpf_host;
    int wpf_recv_port;
    int cpp_recv_port;

    // GStreamer (動的にセクションを保持)
    std::map<std::string, GStreamerConfig> gstreamer_configs;
};

// グローバルな設定オブジェクト
// このオブジェクトは、アプリケーションの起動時に config.ini から読み込まれ、
// 実行中に ConfigSynchronizer によって更新される可能性があります。
extern AppConfig g_config;

// g_config へのスレッドセーフなアクセスのためのミューテックス
extern std::mutex g_config_mutex;

// config.ini を読み込み、g_config グローバル変数を初期化する関数
void loadConfig(const std::string& filename = "config.ini");

// グローバル設定オブジェクトの特定の値を更新するヘルパー関数
// ConfigSynchronizerが受信したキーと値に基づいてg_configを更新するために使用されます。
void updateGConfigValue(const std::string& section, const std::string& key, const std::string& value);


#endif // CONFIG_H