#include "config_sync.h"
#include "config.h"
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>

#define CONFIG_SYNC_PORT 12347 // 設定同期用のTCPポート

// --- グローバル変数 ---
static std::atomic<bool> g_server_running(false);
static std::thread g_server_thread;
static int g_server_socket = -1;
static std::vector<std::thread> g_client_threads;

// --- AppConfig <-> JSON 変換 ---

// AppConfig構造体をJSONオブジェクトに変換
void to_json(nlohmann::json& j, const AppConfig& p) {
    j = nlohmann::json{
        {"pwm", {
            {"pwm_min", p.pwm_min}, {"pwm_neutral", p.pwm_neutral},
            {"pwm_normal_max", p.pwm_normal_max}, {"pwm_boost_max", p.pwm_boost_max},
            {"pwm_frequency", p.pwm_frequency}
        }},
        {"joystick", {{"deadzone", p.joystick_deadzone}}},
        {"led", {
            {"channel", p.led_pwm_channel}, {"on_value", p.led_pwm_on}, {"off_value", p.led_pwm_off}
        }},
        {"thruster_control", {
            {"smoothing_factor_horizontal", p.smoothing_factor_horizontal},
            {"smoothing_factor_vertical", p.smoothing_factor_vertical},
            {"kp_roll", p.kp_roll}, {"kp_yaw", p.kp_yaw},
            {"yaw_threshold_dps", p.yaw_threshold_dps}, {"yaw_gain", p.yaw_gain}
        }},
        {"network", {
            {"recv_port", p.network_recv_port}, {"send_port", p.network_send_port},
            {"client_host", p.client_host}, {"connection_timeout_seconds", p.connection_timeout_seconds}
        }},
        {"application", {
            {"sensor_send_interval", p.sensor_send_interval}, {"loop_delay_us", p.loop_delay_us}
        }},
        {"gstreamer_camera_1", {
            {"device", p.gst1_device}, {"port", p.gst1_port},
            {"width", p.gst1_width}, {"height", p.gst1_height},
            {"framerate_num", p.gst1_framerate_num}, {"framerate_den", p.gst1_framerate_den},
            {"is_h264_native_source", p.gst1_is_h264_native_source},
            {"rtp_payload_type", p.gst1_rtp_payload_type}, {"rtp_config_interval", p.gst1_rtp_config_interval}
        }},
        {"gstreamer_camera_2", {
            {"device", p.gst2_device}, {"port", p.gst2_port},
            {"width", p.gst2_width}, {"height", p.gst2_height},
            {"framerate_num", p.gst2_framerate_num}, {"framerate_den", p.gst2_framerate_den},
            {"is_h264_native_source", p.gst2_is_h264_native_source},
            {"rtp_payload_type", p.gst2_rtp_payload_type}, {"rtp_config_interval", p.gst2_rtp_config_interval},
            {"x264_bitrate", p.gst2_x264_bitrate},
            {"x264_tune", p.gst2_x264_tune}, {"x264_speed_preset", p.gst2_x264_speed_preset}
        }}
    };
}

// JSONオブジェクトからAppConfig構造体に値を設定（部分的な更新に対応）
void from_json(const nlohmann::json& j, AppConfig& p) {
    // キーが存在する場合のみ値を更新する
    auto update_if_present = [](const nlohmann::json& parent, const std::string& key, auto& target) {
        if (parent.contains(key)) {
            parent.at(key).get_to(target);
        }
    };

    if (j.contains("pwm")) {
        const auto& pwm = j.at("pwm");
        update_if_present(pwm, "pwm_min", p.pwm_min);
        update_if_present(pwm, "pwm_neutral", p.pwm_neutral);
        update_if_present(pwm, "pwm_normal_max", p.pwm_normal_max);
        update_if_present(pwm, "pwm_boost_max", p.pwm_boost_max);
        update_if_present(pwm, "pwm_frequency", p.pwm_frequency);
    }
    if (j.contains("joystick")) {
        update_if_present(j.at("joystick"), "deadzone", p.joystick_deadzone);
    }
    if (j.contains("thruster_control")) {
        const auto& tc = j.at("thruster_control");
        update_if_present(tc, "smoothing_factor_horizontal", p.smoothing_factor_horizontal); // "sent"や"_present"のタイポを修正
        update_if_present(tc, "smoothing_factor_vertical", p.smoothing_factor_vertical);   // "sent"や"_present"のタイポを修正
        update_if_present(tc, "kp_roll", p.kp_roll);
        update_if_present(tc, "kp_yaw", p.kp_yaw);
        update_if_present(tc, "yaw_threshold_dps", p.yaw_threshold_dps);
        update_if_present(tc, "yaw_gain", p.yaw_gain);
    }
    if (j.contains("network")) {
        const auto& net = j.at("network");
        update_if_present(net, "recv_port", p.network_recv_port);
        update_if_present(net, "send_port", p.network_send_port);
        update_if_present(net, "client_host", p.client_host);
        update_if_present(net, "connection_timeout_seconds", p.connection_timeout_seconds);
    }
    if (j.contains("application")) {
        const auto& app = j.at("application");
        update_if_present(app, "sensor_send_interval", p.sensor_send_interval);
        update_if_present(app, "loop_delay_us", p.loop_delay_us);
    }
    if (j.contains("gstreamer_camera_1")) {
        const auto& gst1 = j.at("gstreamer_camera_1");
        update_if_present(gst1, "device", p.gst1_device);
        update_if_present(gst1, "port", p.gst1_port);
        update_if_present(gst1, "width", p.gst1_width);
        update_if_present(gst1, "height", p.gst1_height);
        update_if_present(gst1, "framerate_num", p.gst1_framerate_num);
        update_if_present(gst1, "framerate_den", p.gst1_framerate_den);
        update_if_present(gst1, "is_h264_native_source", p.gst1_is_h264_native_source);
        update_if_present(gst1, "rtp_payload_type", p.gst1_rtp_payload_type);
        update_if_present(gst1, "rtp_config_interval", p.gst1_rtp_config_interval);
    }
    if (j.contains("gstreamer_camera_2")) {
        const auto& gst2 = j.at("gstreamer_camera_2");
        update_if_present(gst2, "device", p.gst2_device);
        update_if_present(gst2, "port", p.gst2_port);
        update_if_present(gst2, "width", p.gst2_width);
        update_if_present(gst2, "height", p.gst2_height);
        update_if_present(gst2, "framerate_num", p.gst2_framerate_num);
        update_if_present(gst2, "framerate_den", p.gst2_framerate_den);
        update_if_present(gst2, "is_h264_native_source", p.gst2_is_h264_native_source);
        update_if_present(gst2, "rtp_payload_type", p.gst2_rtp_payload_type);
        update_if_present(gst2, "rtp_config_interval", p.gst2_rtp_config_interval);
        update_if_present(gst2, "x264_bitrate", p.gst2_x264_bitrate);
        update_if_present(gst2, "x264_tune", p.gst2_x264_tune);
        update_if_present(gst2, "x264_speed_preset", p.gst2_x264_speed_preset);
    }
}

// --- TCPサーバーロジック ---

void handle_client(int client_socket) {
    std::cout << "設定同期クライアント接続: " << client_socket << std::endl;

    // 1. 接続時に現在の設定を送信
    try {
        nlohmann::json current_config_json;
        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            to_json(current_config_json, g_config);
        }
        std::string msg = current_config_json.dump() + "\n";
        send(client_socket, msg.c_str(), msg.length(), 0);
    } catch (const std::exception& e) {
        std::cerr << "設定送信エラー: " << e.what() << std::endl;
    }

    // 2. クライアントからの設定変更を受信
    char buffer[4096];
    while (g_server_running) {
        ssize_t len = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
        if (len <= 0) {
            // 接続が切れたかエラー
            break;
        }
        buffer[len] = '\0';

        try {
            nlohmann::json received_json = nlohmann::json::parse(buffer);
            std::cout << "設定更新受信: " << received_json.dump(2) << std::endl;

            // ミューテックスで保護しながらグローバル設定を更新
            {
                std::lock_guard<std::mutex> lock(g_config_mutex);
                from_json(received_json, g_config);
            }
            // TODO: 更新された設定を config.ini に書き戻す処理をここに追加しても良い

        } catch (const nlohmann::json::parse_error& e) {
            std::cerr << "JSONパースエラー: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "設定更新処理エラー: " << e.what() << std::endl;
        }
    }

    close(client_socket);
    std::cout << "設定同期クライアント切断: " << client_socket << std::endl;
}

void server_listen_thread() {
    g_server_running = true;

    sockaddr_in server_addr;
    g_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (g_server_socket < 0) {
        perror("設定同期: ソケット作成失敗");
        return;
    }

    // SO_REUSEADDR オプションを設定して、ポートがすぐに再利用できるようにする
    int opt = 1;
    setsockopt(g_server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(CONFIG_SYNC_PORT);

    if (bind(g_server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("設定同期: バインド失敗");
        close(g_server_socket);
        g_server_socket = -1;
        return;
    }

    if (listen(g_server_socket, 5) < 0) {
        perror("設定同期: listen失敗");
        close(g_server_socket);
        g_server_socket = -1;
        return;
    }

    std::cout << "設定同期サーバー起動 (TCPポート: " << CONFIG_SYNC_PORT << ")" << std::endl;

    while (g_server_running) {
        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(g_server_socket, (struct sockaddr*)&client_addr, &client_len);

        if (client_socket < 0) {
            if (g_server_running) {
                perror("設定同期: accept失敗");
            }
            break; // サーバーが停止された
        }

        // 新しいクライアントを別スレッドで処理
        g_client_threads.emplace_back(handle_client, client_socket);
    }

    close(g_server_socket);
    g_server_socket = -1;
    std::cout << "設定同期サーバーが停止しました。" << std::endl;
}

void start_config_sync_server() {
    if (g_server_running) return;
    g_server_thread = std::thread(server_listen_thread);
}

void stop_config_sync_server() {
    if (!g_server_running) return;

    g_server_running = false;
    // accept()のブロッキングを解除するためにソケットを閉じる
    if (g_server_socket >= 0) {
        shutdown(g_server_socket, SHUT_RDWR);
        close(g_server_socket);
    }

    if (g_server_thread.joinable()) {
        g_server_thread.join();
    }

    for (auto& th : g_client_threads) {
        if (th.joinable()) {
            th.join();
        }
    }
    g_client_threads.clear();
}

