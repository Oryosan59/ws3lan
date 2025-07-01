#include "config.h"
#include <iniparser.h>
#include <iostream>
#include <algorithm> // std::transform
#include <cctype>    // std::tolower

// グローバル設定オブジェクトの定義
AppConfig g_config;
// グローバルミューテックスの定義
std::mutex g_config_mutex;

// 文字列を小文字に変換するヘルパー関数
bool string_to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return str == "true" || str == "1";
}

// GStreamerセクションのキーに基づいてGStreamerConfigを更新するヘルパー関数
void updateGStreamerConfigValue(GStreamerConfig& gst_config, const std::string& key, const std::string& value) {
    if (key == "DEVICE") gst_config.device = value;
    else if (key == "PORT") gst_config.port = std::stoi(value);
    else if (key == "WIDTH") gst_config.width = std::stoi(value);
    else if (key == "HEIGHT") gst_config.height = std::stoi(value);
    else if (key == "FRAMERATE_NUM") gst_config.framerate_num = std::stoi(value);
    else if (key == "FRAMERATE_DEN") gst_config.framerate_den = std::stoi(value);
    else if (key == "IS_H264_NATIVE_SOURCE") gst_config.is_h264_native_source = string_to_bool(value);
    else if (key == "RTP_PAYLOAD_TYPE") gst_config.rtp_payload_type = std::stoi(value);
    else if (key == "RTP_CONFIG_INTERVAL") gst_config.rtp_config_interval = std::stoi(value);
    else if (key == "X264_BITRATE") gst_config.x264_bitrate = std::stoi(value);
    else if (key == "X264_TUNE") gst_config.x264_tune = value;
    else if (key == "X264_SPEED_PRESET") gst_config.x264_speed_preset = value;
}

// ConfigSynchronizerから呼び出されるグローバル設定更新関数
void updateGConfigValue(const std::string& section, const std::string& key, const std::string& value) {
    // 注意: この関数はg_config_mutexによって外部でロックされていることを前提としています
    try {
        if (section == "PWM") {
            if (key == "PWM_MIN") g_config.pwm_min = std::stoi(value);
            else if (key == "PWM_NEUTRAL") g_config.pwm_neutral = std::stoi(value);
            else if (key == "PWM_NORMAL_MAX") g_config.pwm_normal_max = std::stoi(value);
            else if (key == "PWM_BOOST_MAX") g_config.pwm_boost_max = std::stoi(value);
            else if (key == "PWM_FREQUENCY") g_config.pwm_frequency = std::stod(value);
        } else if (section == "JOYSTICK") {
            if (key == "DEADZONE") g_config.joystick_deadzone = std::stoi(value);
        } else if (section == "LED") {
            if (key == "CHANNEL") g_config.led_channel = std::stoi(value);
            else if (key == "ON_VALUE") g_config.led_on_value = std::stoi(value);
            else if (key == "OFF_VALUE") g_config.led_off_value = std::stoi(value);
        } else if (section == "THRUSTER_CONTROL") {
            if (key == "SMOOTHING_FACTOR_HORIZONTAL") g_config.smoothing_factor_horizontal = std::stod(value);
            else if (key == "SMOOTHING_FACTOR_VERTICAL") g_config.smoothing_factor_vertical = std::stod(value);
            else if (key == "KP_ROLL") g_config.kp_roll = std::stod(value);
            else if (key == "KP_YAW") g_config.kp_yaw = std::stod(value);
            else if (key == "YAW_THRESHOLD_DPS") g_config.yaw_threshold_dps = std::stod(value);
            else if (key == "YAW_GAIN") g_config.yaw_gain = std::stod(value);
        } else if (section == "NETWORK") {
            if (key == "RECV_PORT") g_config.network_recv_port = std::stoi(value);
            else if (key == "SEND_PORT") g_config.network_send_port = std::stoi(value);
            else if (key == "CLIENT_HOST") g_config.client_host = value;
            else if (key == "CONNECTION_TIMEOUT_SECONDS") g_config.connection_timeout_seconds = std::stod(value);
        } else if (section == "APPLICATION") {
            if (key == "SENSOR_SEND_INTERVAL") g_config.sensor_send_interval = std::stoul(value);
            else if (key == "LOOP_DELAY_US") g_config.loop_delay_us = std::stoul(value);
        } else if (section == "CONFIG_SYNC") {
            if (key == "WPF_HOST") g_config.wpf_host = value;
            else if (key == "WPF_RECV_PORT") g_config.wpf_recv_port = std::stoi(value);
            else if (key == "CPP_RECV_PORT") g_config.cpp_recv_port = std::stoi(value);
        } else if (section.rfind("GSTREAMER_CAMERA_", 0) == 0) {
            updateGStreamerConfigValue(g_config.gstreamer_configs[section], key, value);
        }
    } catch (const std::invalid_argument& ia) {
        std::cerr << "設定値の変換エラー [" << section << "] " << key << "=" << value << ": " << ia.what() << std::endl;
    } catch (const std::out_of_range& oor) {
        std::cerr << "設定値が範囲外です [" << section << "] " << key << "=" << value << ": " << oor.what() << std::endl;
    }
}

// config.ini を読み込み、g_config グローバル変数を初期化する関数
void loadConfig(const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_config_mutex);

    // --- デフォルト値の設定 ---
    g_config.pwm_min = 1100;
    g_config.pwm_neutral = 1500;
    g_config.pwm_normal_max = 1900;
    g_config.pwm_boost_max = 1900;
    g_config.pwm_frequency = 50.0;
    g_config.joystick_deadzone = 3000;
    g_config.led_channel = 9;
    g_config.led_on_value = 1900;
    g_config.led_off_value = 1100;
    g_config.smoothing_factor_horizontal = 0.15;
    g_config.smoothing_factor_vertical = 0.2;
    g_config.kp_roll = 0.2;
    g_config.kp_yaw = 0.15;
    g_config.yaw_threshold_dps = 2.0;
    g_config.yaw_gain = 50.0;
    g_config.network_recv_port = 12345;
    g_config.network_send_port = 12346;
    g_config.client_host = "192.168.4.10";
    g_config.connection_timeout_seconds = 2.0;
    g_config.sensor_send_interval = 10;
    g_config.loop_delay_us = 10000;
    g_config.wpf_host = "192.168.4.10";
    g_config.wpf_recv_port = 12347;
    g_config.cpp_recv_port = 12348;
    g_config.gstreamer_configs.clear();

    // --- INIファイルからの読み込み ---
    dictionary* ini = iniparser_load(filename.c_str());
    if (ini == NULL) {
        std::cerr << "'" << filename << "' が見つかりません。デフォルト設定を使用します。" << std::endl;
        return;
    }

    // 各セクションから値を読み込む
    int n_sections = iniparser_getnsec(ini);
    for (int i = 0; i < n_sections; i++) {
        const char* section_name_char = iniparser_getsecname(ini, i);
        if (section_name_char == NULL) continue;
        std::string section_name(section_name_char);

        const char** keys = iniparser_getseckeys(ini, section_name_char);
        int n_keys = iniparser_getsecnkeys(ini, section_name_char);

        for (int j = 0; j < n_keys; j++) {
            const char* key_full = keys[j];
            const char* key_simple = strrchr(key_full, ':');
            key_simple = (key_simple) ? key_simple + 1 : key_full;
            
            const char* value_char = iniparser_getstring(ini, key_full, NULL);
            if (value_char) {
                updateGConfigValue(section_name, key_simple, std::string(value_char));
            }
        }
    }

    iniparser_freedict(ini);
    std::cout << "'" << filename << "' から設定を読み込みました。" << std::endl;
}


// ヘルパー関数: 文字列の前後の空白を削除
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (std::string::npos == first) return str;
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(first, (last - first + 1));
}

// ヘルパー関数: 文字列を小文字に変換
static std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return static_cast<unsigned char>(std::tolower(c)); });
    return s;
}

bool loadConfig(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "エラー: 設定ファイル '" << filename << "' を開けません。デフォルト値を使用します。" << std::endl;
        return false;
    }

    std::string line;
    std::string current_section;
    int line_num = 0;

    while (std::getline(file, line)) {
        line_num++;
        line = trim(line);

        if (line.empty() || line[0] == '#' || line[0] == ';') {
            // 空行またはコメント行
            continue;
        }

        if (line[0] == '[' && line.back() == ']') {
            // セクションヘッダー
            current_section = toLower(line.substr(1, line.length() - 2));
            continue;
        }

        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) {
            std::cerr << "警告: " << filename << " の " << line_num << " 行目: '=' が見つかりません。スキップします。" << std::endl;
            continue;
        }

        std::string key = toLower(trim(line.substr(0, eq_pos)));
        std::string value = trim(line.substr(eq_pos + 1));

        // 設定値のパースと適用
        try {
            if (current_section == "pwm") {
                if (key == "pwm_min") g_config.pwm_min = std::stoi(value);
                else if (key == "pwm_neutral") g_config.pwm_neutral = std::stoi(value);
                else if (key == "pwm_normal_max") g_config.pwm_normal_max = std::stoi(value);
                else if (key == "pwm_boost_max") g_config.pwm_boost_max = std::stoi(value);
                else if (key == "pwm_frequency") g_config.pwm_frequency = std::stof(value);
            } else if (current_section == "joystick") {
                if (key == "deadzone") g_config.joystick_deadzone = std::stoi(value);
            } else if (current_section == "led") {
                if (key == "channel") g_config.led_pwm_channel = std::stoi(value);
                else if (key == "on_value") g_config.led_pwm_on = std::stoi(value);
                else if (key == "off_value") g_config.led_pwm_off = std::stoi(value);
            } else if (current_section == "thruster_control") {
                if (key == "smoothing_factor_horizontal") g_config.smoothing_factor_horizontal = std::stof(value);
                else if (key == "smoothing_factor_vertical") g_config.smoothing_factor_vertical = std::stof(value);
                else if (key == "kp_roll") g_config.kp_roll = std::stof(value);
                else if (key == "kp_yaw") g_config.kp_yaw = std::stof(value);
                else if (key == "yaw_threshold_dps") g_config.yaw_threshold_dps = std::stof(value);
                else if (key == "yaw_gain") g_config.yaw_gain = std::stof(value);
            } else if (current_section == "network") {
                if (key == "recv_port") g_config.network_recv_port = std::stoi(value);
                else if (key == "send_port") g_config.network_send_port = std::stoi(value);
                else if (key == "client_host") g_config.client_host = value;
                else if (key == "connection_timeout_seconds") g_config.connection_timeout_seconds = std::stod(value);
            } else if (current_section == "application") {
                if (key == "sensor_send_interval") g_config.sensor_send_interval = std::stoul(value);
                else if (key == "loop_delay_us") g_config.loop_delay_us = std::stoul(value);
            } else if (current_section == "gstreamer_camera_1") {
                if (key == "port") g_config.gst1_port = std::stoi(value);
                else if (key == "height") g_config.gst1_height = std::stoi(value); else if (key == "framerate_num") g_config.gst1_framerate_num = std::stoi(value);
                else if (key == "framerate_den") g_config.gst1_framerate_den = std::stoi(value); else if (key == "is_h264_native_source") g_config.gst1_is_h264_native_source = (toLower(value) == "true");
                else if (key == "rtp_payload_type") g_config.gst1_rtp_payload_type = std::stoi(value); else if (key == "rtp_config_interval") g_config.gst1_rtp_config_interval = std::stoi(value);
            } else if (current_section == "gstreamer_camera_2") {
                if (key == "port") g_config.gst2_port = std::stoi(value);
                else if (key == "height") g_config.gst2_height = std::stoi(value); else if (key == "framerate_num") g_config.gst2_framerate_num = std::stoi(value);
                else if (key == "framerate_den") g_config.gst2_framerate_den = std::stoi(value); else if (key == "is_h264_native_source") g_config.gst2_is_h264_native_source = (toLower(value) == "true");
                else if (key == "rtp_payload_type") g_config.gst2_rtp_payload_type = std::stoi(value); else if (key == "rtp_config_interval") g_config.gst2_rtp_config_interval = std::stoi(value);
                else if (key == "x264_bitrate") g_config.gst2_x264_bitrate = std::stoi(value); else if (key == "x264_tune") g_config.gst2_x264_tune = value;
                else if (key == "x264_speed_preset") g_config.gst2_x264_speed_preset = value;
            } else {
                std::cerr << "警告: " << filename << " の " << line_num << " 行目: 不明なセクションまたはキー [" << current_section << "] " << key << "=" << value << std::endl;
            }
        } catch (const std::invalid_argument& e) {
            std::cerr << "エラー: " << filename << " の " << line_num << " 行目: 数値変換エラー (" << key << "=" << value << ") - " << e.what() << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "エラー: " << filename << " の " << line_num << " 行目: 数値が範囲外 (" << key << "=" << value << ") - " << e.what() << std::endl;
        }
    }
    std::cout << "設定ファイル '" << filename << "' を読み込みました。" << std::endl;
    return true;
}
