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
bool running = true; // メインループの実行フラグ

// --- シグナルハンドラ ---
void handle_signal(int signum) {
    std::cout << "
シグナル " << signum << " を受信。終了処理を開始します..." << std::endl;
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
    init(); // Navigator ハードウェアライブラリの初期化

    NetworkContext net_ctx;
    if (!network_init(&net_ctx))
    {
        std::cerr << "ネットワーク初期化失敗。終了します。" << std::endl;
        stop_config_synchronizer(); // 終了前に同期スレッドを停止
        return -1;
    }

    if (!thruster_init())
    {
        std::cerr << "スラスター初期化失敗。終了します。" << std::endl;
        network_close(&net_ctx);
        stop_config_synchronizer();
        return -1;
    }

    if (!start_gstreamer_pipelines())
    {
        std::cerr << "GStreamerパイプラインの起動に失敗しました。処理を続行します..." << std::endl;
    }

    // --- メインループ ---
    GamepadData latest_gamepad_data;
    char recv_buffer[NET_BUFFER_SIZE];
    AxisData current_gyro_data = {0.0f, 0.0f, 0.0f};
    char sensor_buffer[SENSOR_BUFFER_SIZE];
    unsigned int loop_counter = 0;
    bool currently_in_failsafe = true;

    int initial_pwm;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex);
        initial_pwm = g_config.pwm_min;
    }
    std::cout << "メインループ開始。" << std::endl;
    std::cout << "クライアントからの最初のデータ受信を待機しています... (スラスターはPWM: " << initial_pwm << ")" << std::endl;
    thruster_set_all_pwm(initial_pwm);

    while (running)
    {
        struct timeval current_time_tv;
        gettimeofday(&current_time_tv, NULL);

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

        // 1. ネットワーク接続状態チェック
        double time_since_last_packet = 0.0;
        if (net_ctx.client_addr_known)
        {
            time_since_last_packet = (current_time_tv.tv_sec - net_ctx.last_successful_recv_time.tv_sec) +
                                     (current_time_tv.tv_usec - net_ctx.last_successful_recv_time.tv_usec) / 1000000.0;
        }

        // 2. ゲームパッドデータ受信
        ssize_t recv_len = network_receive(&net_ctx, recv_buffer, sizeof(recv_buffer));
        bool just_received_packet = (recv_len > 0);

        if (just_received_packet)
        {
            if (currently_in_failsafe)
            {
                std::cout << "接続確立/再確立。通常動作を再開します。" << std::endl;
                currently_in_failsafe = false;
            }
            std::string received_str(recv_buffer, recv_len);
            latest_gamepad_data = parseGamepadData(received_str);
        }
        else
        {
            if (net_ctx.client_addr_known && time_since_last_packet > current_timeout_seconds)
            {
                if (!currently_in_failsafe)
                {
                    std::cout << "接続がタイムアウトしました。フェイルセーフモード (スラスターPWM: " << current_pwm_min << ") に移行します。" << std::endl;
                    thruster_set_all_pwm(current_pwm_min);
                    latest_gamepad_data = GamepadData{};
                    currently_in_failsafe = true;
                    // フェイルセーフで即時終了する代わりに、再接続を待つ
                }
            }
            if (recv_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                std::cerr << "致命的な受信エラー。ループを継続します..." << std::endl;
            }
        }

        // 3. 制御ロジック
        if (!currently_in_failsafe && running)
        {
            current_gyro_data = read_gyro();
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
                    std::cerr << "センサーデータの読み取り/フォーマットに失敗。" << std::endl;
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
    std::cout << "クリーンアップ処理を開始します..." << std::endl;
    stop_config_synchronizer(); // 設定同期スレッドを停止
    thruster_disable();
    network_close(&net_ctx);
    stop_gstreamer_pipelines();
    std::cout << "プログラム終了。" << std::endl;
    return 0;
}
