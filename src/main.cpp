// --- インクルード ---
#include "network.h"          // ネットワーク通信関連 (UDP送受信)
#include "gamepad.h"          // ゲームパッドデータ構造体とパース関数
#include "thruster_control.h" // スラスター制御関連
#include "sensor_data.h"      // センサーデータ読み取り・フォーマット関連
#include "gstPipeline.h"      // GStreamerパイプライン起動用
#include "config.h"           // 設定ファイル読み込みとグローバル設定オブジェクト
#include "ConfigSynchronizer.h" // 設定同期機能

#include <iostream> // 標準入出力 (std::cout, std::cerr)
#include <unistd.h> // POSIX API (usleep)
#include <string.h> // strlen
#include <sys/time.h> // gettimeofday
#include <signal.h> // シグナルハンドリング

// --- グローバル変数 ---
volatile bool running = true; // メインループの実行フラグ (volatileを付与)

// --- シグナルハンドラ ---
void handle_signal(int signum) {
    // この関数はシグナルハンドラ内なので、非同期シグナルセーフな関数のみ使用するのが望ましい
    // ここでは単純なフラグ設定に留める
    running = false;
}

// --- メイン関数 ---
int main()
{
    // シグナルハンドラの設定 (Ctrl+Cなどで安全に終了するため)
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("Navigator C++ Control Application\n");

    // --- 初期化 ---
    // 設定の初期読み込み (ConfigSynchronizer起動前)
    loadConfig("config.ini");

    // 設定同期スレッドの起動
    start_config_synchronizer("config.ini");

    printf("Initiating navigator module.\n");
    // init(); // BlueRobotics Navigatorライブラリの初期化関数。実際のハードウェアに合わせて有効化してください。

    NetworkContext net_ctx;
    if (!network_init(&net_ctx))
    {
        std::cerr << "Network initialization failed. Exiting." << std::endl;
        stop_config_synchronizer(); // 終了前に同期スレッドを停止
        return -1;
    }

    if (!thruster_init())
    {
        std::cerr << "Thruster initialization failed. Exiting." << std::endl;
        network_close(&net_ctx);
        stop_config_synchronizer();
        return -1;
    }

    // GStreamerパイプラインの起動
    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        if (!start_gstreamer_pipelines(g_config))
        {
            std::cerr << "Failed to start GStreamer pipelines. Continuing without video streams..." << std::endl;
        }
    }

    // --- メインループ ---
    GamepadData latest_gamepad_data;
    char recv_buffer[NET_BUFFER_SIZE];
    AxisData current_gyro_data = {0.0f, 0.0f, 0.0f};
    char sensor_buffer[SENSOR_BUFFER_SIZE];
    unsigned int loop_counter = 0;
    bool currently_in_failsafe = true;

    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        std::cout << "Main loop started." << std::endl;
        std::cout << "Waiting for first data from client... (Thrusters set to PWM: " << g_config.pwm_min << ")" << std::endl;
        thruster_set_all_pwm(g_config.pwm_min);
    }

    while (running)
    {
        // 設定値をスレッドセーフに取得
        double current_timeout_seconds;
        unsigned int current_sensor_interval;
        unsigned int current_loop_delay;
        int current_pwm_min;
        {
            std::lock_guard<std::mutex> lock(g_config_mutex);
            current_timeout_seconds = g_config.connection_timeout_seconds;
            current_sensor_interval = g_config.sensor_send_interval;
            current_loop_delay = g_config.loop_delay_us;
            current_pwm_min = g_config.pwm_min;
        }

        // 1. ゲームパッドデータ受信
        ssize_t recv_len = network_receive(&net_ctx, recv_buffer, sizeof(recv_buffer));
        bool just_received_packet = (recv_len > 0);

        if (just_received_packet)
        {
            if (currently_in_failsafe)
            {
                std::cout << "Connection established/re-established. Resuming normal operation." << std::endl;
                currently_in_failsafe = false;
            }
            std::string received_str(recv_buffer, recv_len);
            latest_gamepad_data = parseGamepadData(received_str);
        }
        else
        {
            struct timeval current_time_tv;
            gettimeofday(&current_time_tv, NULL);
            double time_since_last_packet = 0.0;
            if (net_ctx.client_addr_known)
            {
                time_since_last_packet = (current_time_tv.tv_sec - net_ctx.last_successful_recv_time.tv_sec) +
                                         (current_time_tv.tv_usec - net_ctx.last_successful_recv_time.tv_usec) / 1000000.0;
            }

            if (net_ctx.client_addr_known && time_since_last_packet > current_timeout_seconds)
            {
                if (!currently_in_failsafe)
                {
                    std::cout << "Connection timed out. Entering failsafe mode (Thrusters PWM: " << current_pwm_min << ")" << std::endl;
                    thruster_set_all_pwm(current_pwm_min);
                    latest_gamepad_data = GamepadData{};
                    currently_in_failsafe = true;
                }
            }
        }

        // 3. 制御ロジック
        if (!currently_in_failsafe && running)
        {
            // current_gyro_data = read_gyro(); // 実際のハードウェアに合わせて有効化
            thruster_update(latest_gamepad_data, current_gyro_data);

            if (loop_counter >= current_sensor_interval)
            {
                loop_counter = 0;
                if (read_and_format_sensor_data(sensor_buffer, sizeof(sensor_buffer)))
                {
                    network_send(&net_ctx, sensor_buffer, strlen(sensor_buffer));
                }
                else
                {
                    std::cerr << "Failed to read/format sensor data." << std::endl;
                }
            }
            else
            {
                loop_counter++;
            }
        }
        else
        {
            loop_counter = 0;
        }

        // 4. ループ待機
        usleep(current_loop_delay);
    }

    // --- クリーンアップ ---
    std::cout << "\nInitiating cleanup..." << std::endl;
    stop_config_synchronizer(); // 設定同期スレッドを停止
    thruster_disable();
    network_close(&net_ctx);
    stop_gstreamer_pipelines();
    std::cout << "Program finished." << std::endl;
    return 0;
}