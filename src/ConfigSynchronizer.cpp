// ConfigSynchronizer.cpp - ライブラリ版
//
// 目的:
// 1. TCPクライアントとして、現在の設定をWPFアプリケーションに送信する
// 2. TCPサーバーとして、WPFアプリケーションからの設定変更を待ち受け、動的に反映する
//
// このファイルは、main.cppから呼び出されることを想定しており、
// start_config_synchronizer() でバックグラウンドスレッドを開始し、
// stop_config_synchronizer() で安全に停止します。

#include "ConfigSynchronizer.h"
#include "config.h" // g_config と g_config_mutex を共有するため

#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <vector>
#include <sstream>
#include <fstream>
#include <chrono>
#include <atomic>
#include <algorithm>

// Linux用のソケットライブラリ
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>

// --- グローバル変数 ---
static std::thread g_receiver_thread;
static std::atomic<bool> g_shutdown_flag{false};

// --- プロトタイプ宣言 ---
static void receive_config_updates(const std::string& config_path);
static void send_config_to_wpf();
static void save_config_to_file(const std::string& filename);
static void update_global_config_from_map(const std::map<std::string, std::map<std::string, std::string>>& new_config_data);
static std::map<std::string, std::map<std::string, std::string>> parse_config_from_string(const std::string& data);

/**
 * @brief 現在のグローバル設定をファイルに保存する
 */
static void save_config_to_file(const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_config_mutex);
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "エラー: 設定ファイル " << filename << " を書き込み用に開けませんでした。\n";
        return;
    }

    file << "# Navigator C++制御アプリケーションの設定ファイル\n";
    file << "# ConfigSynchronizerによって自動更新されました\n\n";

    file << "[PWM]\n";
    file << "PWM_MIN=" << g_config.pwm_min << "\n";
    file << "PWM_NEUTRAL=" << g_config.pwm_neutral << "\n";
    file << "PWM_NORMAL_MAX=" << g_config.pwm_normal_max << "\n";
    file << "PWM_BOOST_MAX=" << g_config.pwm_boost_max << "\n";
    file << "PWM_FREQUENCY=" << g_config.pwm_frequency << "\n\n";

    file << "[JOYSTICK]\n";
    file << "DEADZONE=" << g_config.joystick_deadzone << "\n\n";
    
    file << "[LED]\n";
    file << "CHANNEL=" << g_config.led_channel << "\n";
    file << "ON_VALUE=" << g_config.led_on_value << "\n";
    file << "OFF_VALUE=" << g_config.led_off_value << "\n\n";

    file << "[THRUSTER_CONTROL]\n";
    file << "SMOOTHING_FACTOR_HORIZONTAL=" << g_config.smoothing_factor_horizontal << "\n";
    file << "SMOOTHING_FACTOR_VERTICAL=" << g_config.smoothing_factor_vertical << "\n";
    file << "KP_ROLL=" << g_config.kp_roll << "\n";
    file << "KP_YAW=" << g_config.kp_yaw << "\n";
    file << "YAW_THRESHOLD_DPS=" << g_config.yaw_threshold_dps << "\n";
    file << "YAW_GAIN=" << g_config.yaw_gain << "\n\n";

    file << "[NETWORK]\n";
    file << "RECV_PORT=" << g_config.network_recv_port << "\n";
    file << "SEND_PORT=" << g_config.network_send_port << "\n";
    file << "CLIENT_HOST=" << g_config.client_host << "\n";
    file << "CONNECTION_TIMEOUT_SECONDS=" << g_config.connection_timeout_seconds << "\n\n";

    file << "[APPLICATION]\n";
    file << "SENSOR_SEND_INTERVAL=" << g_config.sensor_send_interval << "\n";
    file << "LOOP_DELAY_US=" << g_config.loop_delay_us << "\n\n";

    file << "[CONFIG_SYNC]\n";
    file << "WPF_HOST=" << g_config.wpf_host << "\n";
    file << "WPF_RECV_PORT=" << g_config.wpf_recv_port << "\n";
    file << "CPP_RECV_PORT=" << g_config.cpp_recv_port << "\n\n";
    
    for(const auto& pair : g_config.gstreamer_configs){
        const std::string& section_name = pair.first;
        const GStreamerConfig& gst_conf = pair.second;
        file << "[" << section_name << "]\n";
        file << "DEVICE=" << gst_conf.device << "\n";
        file << "PORT=" << gst_conf.port << "\n";
        file << "WIDTH=" << gst_conf.width << "\n";
        file << "HEIGHT=" << gst_conf.height << "\n";
        file << "FRAMERATE_NUM=" << gst_conf.framerate_num << "\n";
        file << "FRAMERATE_DEN=" << gst_conf.framerate_den << "\n";
        file << "IS_H264_NATIVE_SOURCE=" << (gst_conf.is_h264_native_source ? "true" : "false") << "\n";
        file << "RTP_PAYLOAD_TYPE=" << gst_conf.rtp_payload_type << "\n";
        file << "RTP_CONFIG_INTERVAL=" << gst_conf.rtp_config_interval << "\n";
        file << "X264_BITRATE=" << gst_conf.x264_bitrate << "\n";
        file << "X264_TUNE=" << gst_conf.x264_tune << "\n";
        file << "X264_SPEED_PRESET=" << gst_conf.x264_speed_preset << "\n\n";
    }

    file.close();
    std::cout << "設定を " << filename << " に保存しました。\n";
}

/**
 * @brief 現在のグローバル設定をシリアライズして文字列にする
 */
static std::string serialize_config() {
    std::lock_guard<std::mutex> lock(g_config_mutex);
    std::stringstream content_ss;

    content_ss << "[PWM]PWM_MIN=" << g_config.pwm_min << "\n";
    content_ss << "[PWM]PWM_NEUTRAL=" << g_config.pwm_neutral << "\n";
    content_ss << "[PWM]PWM_NORMAL_MAX=" << g_config.pwm_normal_max << "\n";
    content_ss << "[PWM]PWM_BOOST_MAX=" << g_config.pwm_boost_max << "\n";
    content_ss << "[PWM]PWM_FREQUENCY=" << g_config.pwm_frequency << "\n";
    content_ss << "[JOYSTICK]DEADZONE=" << g_config.joystick_deadzone << "\n";
    content_ss << "[LED]CHANNEL=" << g_config.led_channel << "\n";
    content_ss << "[LED]ON_VALUE=" << g_config.led_on_value << "\n";
    content_ss << "[LED]OFF_VALUE=" << g_config.led_off_value << "\n";
    content_ss << "[THRUSTER_CONTROL]SMOOTHING_FACTOR_HORIZONTAL=" << g_config.smoothing_factor_horizontal << "\n";
    content_ss << "[THRUSTER_CONTROL]SMOOTHING_FACTOR_VERTICAL=" << g_config.smoothing_factor_vertical << "\n";
    content_ss << "[THRUSTER_CONTROL]KP_ROLL=" << g_config.kp_roll << "\n";
    content_ss << "[THRUSTER_CONTROL]KP_YAW=" << g_config.kp_yaw << "\n";
    content_ss << "[THRUSTER_CONTROL]YAW_THRESHOLD_DPS=" << g_config.yaw_threshold_dps << "\n";
    content_ss << "[THRUSTER_CONTROL]YAW_GAIN=" << g_config.yaw_gain << "\n";
    content_ss << "[NETWORK]RECV_PORT=" << g_config.network_recv_port << "\n";
    content_ss << "[NETWORK]SEND_PORT=" << g_config.network_send_port << "\n";
    content_ss << "[NETWORK]CLIENT_HOST=" << g_config.client_host << "\n";
    content_ss << "[NETWORK]CONNECTION_TIMEOUT_SECONDS=" << g_config.connection_timeout_seconds << "\n";
    content_ss << "[APPLICATION]SENSOR_SEND_INTERVAL=" << g_config.sensor_send_interval << "\n";
    content_ss << "[APPLICATION]LOOP_DELAY_US=" << g_config.loop_delay_us << "\n";
    content_ss << "[CONFIG_SYNC]WPF_HOST=" << g_config.wpf_host << "\n";
    content_ss << "[CONFIG_SYNC]WPF_RECV_PORT=" << g_config.wpf_recv_port << "\n";
    content_ss << "[CONFIG_SYNC]CPP_RECV_PORT=" << g_config.cpp_recv_port << "\n";

    for(const auto& pair : g_config.gstreamer_configs){
        const std::string& section_name = pair.first;
        const GStreamerConfig& gst_conf = pair.second;
        content_ss << "[" << section_name << "]DEVICE=" << gst_conf.device << "\n";
        content_ss << "[" << section_name << "]PORT=" << gst_conf.port << "\n";
        content_ss << "[" << section_name << "]WIDTH=" << gst_conf.width << "\n";
        content_ss << "[" << section_name << "]HEIGHT=" << gst_conf.height << "\n";
        content_ss << "[" << section_name << "]FRAMERATE_NUM=" << gst_conf.framerate_num << "\n";
        content_ss << "[" << section_name << "]FRAMERATE_DEN=" << gst_conf.framerate_den << "\n";
        content_ss << "[" << section_name << "]IS_H264_NATIVE_SOURCE=" << (gst_conf.is_h264_native_source ? "true" : "false") << "\n";
        content_ss << "[" << section_name << "]RTP_PAYLOAD_TYPE=" << gst_conf.rtp_payload_type << "\n";
        content_ss << "[" << section_name << "]RTP_CONFIG_INTERVAL=" << gst_conf.rtp_config_interval << "\n";
        content_ss << "[" << section_name << "]X264_BITRATE=" << gst_conf.x264_bitrate << "\n";
        content_ss << "[" << section_name << "]X264_TUNE=" << gst_conf.x264_tune << "\n";
        content_ss << "[" << section_name << "]X264_SPEED_PRESET=" << gst_conf.x264_speed_preset << "\n";
    }

    std::string content = content_ss.str();
    std::stringstream ss;
    ss << content.length() << "\n" << content;
    return ss.str();
}

/**
 * @brief 受信した文字列から設定をパースし、一時マップに格納する
 */
static std::map<std::string, std::map<std::string, std::string>> parse_config_from_string(const std::string& data) {
    std::map<std::string, std::map<std::string, std::string>> parsed_data;
    std::stringstream ss(data);
    std::string line;

    while (std::getline(ss, line)) {
        if (line.empty() || line[0] != '[') continue;

        size_t section_end = line.find(']');
        size_t equals_pos = line.find('=', section_end);

        if (section_end != std::string::npos && equals_pos != std::string::npos) {
            std::string section = line.substr(1, section_end - 1);
            std::string key = line.substr(section_end + 1, equals_pos - (section_end + 1));
            std::string value = line.substr(equals_pos + 1);
            value.erase(value.find_last_not_of(" \n\r\t") + 1);
            parsed_data[section][key] = value;
        }
    }
    return parsed_data;
}

/**
 * @brief パースされた設定マップからグローバル設定オブジェクトを更新する
 */
static void update_global_config_from_map(const std::map<std::string, std::map<std::string, std::string>>& new_config_data) {
    std::lock_guard<std::mutex> lock(g_config_mutex);

    for (const auto& section_pair : new_config_data) {
        const std::string& section = section_pair.first;
        const auto& key_values = section_pair.second;

        for (const auto& kv_pair : key_values) {
            const std::string& key = kv_pair.first;
            const std::string& value = kv_pair.second;
            updateGConfigValue(section, key, value);
        }
    }
    std::cout << "グローバル設定オブジェクトを更新しました。\n";
}

/**
 * @brief ソケットのノンブロッキングモードを設定する
 */
static bool set_socket_non_blocking(int sock, bool non_blocking) {
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1) return false;
    flags = non_blocking ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    return fcntl(sock, F_SETFL, flags) != -1;
}

/**
 * @brief WPFアプリケーションに現在の設定を送信する
 */
static void send_config_to_wpf() {
    std::string host;
    int port;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        host = g_config.wpf_host;
        port = g_config.wpf_recv_port;
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "エラー: 送信用ソケットを作成できませんでした。 " << strerror(errno) << std::endl;
        return;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "エラー: 不正なIPアドレス: " << host << std::endl;
        close(sock);
        return;
    }

    set_socket_non_blocking(sock, true);
    connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));

    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(sock, &fdset);
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;

    if (select(sock + 1, NULL, &fdset, NULL, &timeout) == 1) {
        int so_error;
        socklen_t len = sizeof(so_error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if (so_error == 0) {
            std::cout << "WPFアプリケーションに接続しました。設定を送信します...\n";
            std::string config_str = serialize_config();
            send(sock, config_str.c_str(), config_str.length(), 0);
            std::cout << "設定を送信しました。\n";
        } else {
            // 接続失敗はエラーとして出力するが、プログラムは続行
            // std::cerr << "エラー: WPFへの接続に失敗しました。 " << strerror(so_error) << std::endl;
        }
    } else {
        // タイムアウトも同様
        // std::cerr << "エラー: WPFへの接続がタイムアウトしました。\n";
    }

    close(sock);
}

/**
 * @brief 既存のソケットを通じて現在の設定を送信する
 */
static void send_config_on_existing_socket(int sock) {
    std::string config_str = serialize_config();
    send(sock, config_str.c_str(), config_str.length(), 0);
    std::cout << "現在の設定を返信しました。\n";
}

/**
 * @brief クライアントからの接続を処理し、完全なメッセージを受信する
 */
static void handle_client_connection(int client_sock, const std::string& config_path) {
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    try {
        std::string header;
        char c;
        while (recv(client_sock, &c, 1, 0) > 0 && !g_shutdown_flag.load()) {
            if (c == '\n') break;
            header += c;
        }

        if (header.empty() || g_shutdown_flag.load()) {
            close(client_sock);
            return;
        }

        size_t expected_length = std::stoull(header);
        if (expected_length == 0) {
            std::cout << "\nWPFから設定要求を受信。現在の設定を返信します。\n";
            send_config_on_existing_socket(client_sock);
            close(client_sock);
            return;
        }
        
        const size_t MAX_MESSAGE_SIZE = 1024 * 1024; // 1MB
        if (expected_length > MAX_MESSAGE_SIZE) {
             std::cerr << "エラー: メッセージサイズが大きすぎます: " << expected_length << " bytes\n";
             close(client_sock);
             return;
        }

        std::vector<char> buffer(expected_length);
        size_t total_received = 0;
        while (total_received < expected_length && !g_shutdown_flag.load()) {
            ssize_t bytes_received = recv(client_sock, buffer.data() + total_received, expected_length - total_received, 0);
            if (bytes_received <= 0) {
                std::cerr << "エラー: データ受信中にエラーが発生しました。\n";
                close(client_sock);
                return;
            }
            total_received += bytes_received;
        }

        if (!g_shutdown_flag.load()) {
            std::cout << "\nWPFから設定データを受信しました (" << total_received << " バイト)\n";
            std::string received_data(buffer.begin(), buffer.end());
            auto parsed_map = parse_config_from_string(received_data);
            update_global_config_from_map(parsed_map);
            save_config_to_file(config_path);
        }

    } catch (const std::exception& e) {
        std::cerr << "エラー: クライアント接続処理中に例外が発生しました: " << e.what() << std::endl;
    }
    
    close(client_sock);
}

/**
 * @brief WPFからの設定更新を待ち受けるサーバーとして動作する (別スレッドで実行)
 */
static void receive_config_updates(const std::string& config_path) {
    int port;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        port = g_config.cpp_recv_port;
    }

    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        std::cerr << "エラー: 受信用ソケットを作成できませんでした。" << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "エラー: ポート " << port << " にバインドできませんでした。" << std::endl;
        close(listen_sock);
        return;
    }

    if (listen(listen_sock, 5) < 0) {
        std::cerr << "エラー: listenに失敗しました。" << std::endl;
        close(listen_sock);
        return;
    }

    std::cout << "ConfigSynchronizer: ポート " << port << " でWPFからの設定更新を待機しています...\n";

    while (!g_shutdown_flag.load()) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listen_sock, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        int activity = select(listen_sock + 1, &readfds, nullptr, nullptr, &timeout);
        
        if (activity < 0 && errno != EINTR) {
            std::cerr << "エラー: selectに失敗しました。" << std::endl;
            break;
        }
        
        if (activity > 0 && FD_ISSET(listen_sock, &readfds)) {
            int client_sock = accept(listen_sock, nullptr, nullptr);
            if (client_sock >= 0) {
                handle_client_connection(client_sock, config_path);
            }
        }
    }

    close(listen_sock);
    std::cout << "ConfigSynchronizer: 受信スレッドを終了しました。\n";
}

// --- 公開関数 ---

/**
 * @brief ConfigSynchronizerを初期化し、バックグラウンドで起動する
 */
void start_config_synchronizer(const std::string& config_path) {
    std::cout << "ConfigSynchronizerを起動します...\n";
    
    // 受信スレッドを開始
    g_receiver_thread = std::thread(receive_config_updates, config_path);

    // 少し待ってから、最初の設定をWPFに送信
    std::this_thread::sleep_for(std::chrono::seconds(1));
    send_config_to_wpf();

    std::cout << "ConfigSynchronizerがバックグラウンドで実行中です。\n";
}

/**
 * @brief ConfigSynchronizerを停止する
 */
void stop_config_synchronizer() {
    std::cout << "ConfigSynchronizerを停止します...\n";
    g_shutdown_flag.store(true);
    if (g_receiver_thread.joinable()) {
        g_receiver_thread.join();
    }
    std::cout << "ConfigSynchronizerが停止しました。\n";
}