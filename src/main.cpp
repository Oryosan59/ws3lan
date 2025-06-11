// --- インクルード ---
#include "network.h"          // ネットワーク通信関連 (UDP送受信)
#include "gamepad.h"          // ゲームパッドデータ構造体とパース関数
#include "thruster_control.h" // スラスター制御関連
#include "sensor_data.h"      // センサーデータ読み取り・フォーマット関連
#include "gstPipeline.h"      // GStreamerパイプライン起動用

#include <iostream> // 標準入出力 (std::cout, std::cerr)
#include <unistd.h> // POSIX API (usleep)
#include <string.h> // 文字列操作 (strlen)
#include <sys/time.h> // gettimeofday を使用するため

// --- 定数 ---
const double CONNECTION_TIMEOUT_SECONDS = 0.2; // 接続タイムアウトまでの秒数 (0.2秒)


// --- メイン関数 ---
int main()
{
    printf("Navigator C++ Control Application\n");

    // --- 初期化 ---
    printf("Initiating navigator module.\n");
    init(); // Navigator ハードウェアライブラリの初期化 (bindings.h 経由)

    // ネットワークコンテキストの初期化
    NetworkContext net_ctx;
    if (!network_init(&net_ctx, DEFAULT_RECV_PORT, DEFAULT_SEND_PORT))
    {
        std::cerr << "ネットワーク初期化失敗。終了します。" << std::endl;
        return -1;
    }

    // スラスター制御の初期化
    if (!thruster_init())
    {
        std::cerr << "スラスター初期化失敗。終了します。" << std::endl;
        network_close(&net_ctx); // ネットワークリソースを解放
        return -1;
    }

    // GStreamerパイプラインの起動
    if (!start_gstreamer_pipelines())
    {
        std::cerr << "GStreamerパイプラインの起動に失敗しました。処理を続行します..." << std::endl;
        // パイプライン起動失敗は致命的ではないかもしれないので、ここでは続行
    }

    // --- メインループ ---
    GamepadData latest_gamepad_data;                 // 最後に受信した有効なゲームパッドデータを保持
    char recv_buffer[NET_BUFFER_SIZE];               // UDP受信バッファ
    AxisData current_gyro_data = {0.0f, 0.0f, 0.0f}; // 最新のジャイロデータを保持
    char sensor_buffer[SENSOR_BUFFER_SIZE];          // センサーデータ送信用文字列バッファ
    unsigned int loop_counter = 0;                   // センサーデータ送信間隔制御用カウンター
    const unsigned int SENSOR_SEND_INTERVAL = 10;    // センサーデータを送信するループ間隔 (100Hzループで10回 -> 10Hz)
    bool running = true;                             // メインループの実行フラグ

    bool currently_in_failsafe = true; // 初期状態はフェイルセーフ (最初の接続を待つ)

    std::cout << "メインループ開始。Startボタンで終了。" << std::endl;
    std::cout << "クライアントからの最初のデータ受信を待機しています... (スラスターはPWM: " << PWM_MIN << ")" << std::endl;
    thruster_set_all_pwm(PWM_MIN); // プログラム開始時にスラスターを安全な状態に設定

    // running フラグが true の間、ループを継続
    while (running)
    {
        struct timeval current_time_tv;
        gettimeofday(&current_time_tv, NULL);

        // 1. ネットワーク接続状態チェック (最後にパケットを受信してからの時間)
        double time_since_last_packet = 0.0;
        // net_ctx.client_addr_known は、network_receive内で最初の有効なパケット受信時にtrueになる
        if (net_ctx.client_addr_known)
        {
            time_since_last_packet = (current_time_tv.tv_sec - net_ctx.last_successful_recv_time.tv_sec) +
                                     (current_time_tv.tv_usec - net_ctx.last_successful_recv_time.tv_usec) / 1000000.0;
        }

        // 2. ゲームパッドデータ受信 (network_receive は net_ctx.last_successful_recv_time を更新)
        ssize_t recv_len = network_receive(&net_ctx, recv_buffer, sizeof(recv_buffer));
        bool just_received_packet = (recv_len > 0);


        if (just_received_packet)
        {
            if (currently_in_failsafe) // フェイルセーフ状態からの復帰
            {
                std::cout << "接続確立/再確立。通常動作を再開します。" << std::endl;
                currently_in_failsafe = false;
                // 必要であれば、ここで thruster_init() を呼び出すなど復帰処理を追加
            }
            std::string received_str(recv_buffer, recv_len); // 受信した長さで文字列を作成

            latest_gamepad_data = parseGamepadData(received_str); // 受信文字列をパース
            // std::cout << "受信: " << received_str << std::endl; // Debug
        }
        else // 今回のループではパケット受信なし
        {
            // 接続が一度確立された後でタイムアウトした場合
            if (net_ctx.client_addr_known && time_since_last_packet > CONNECTION_TIMEOUT_SECONDS)
            {
                if (!currently_in_failsafe)
                {
                    std::cout << "接続がタイムアウトしました。フェイルセーフモード (スラスターPWM: " << PWM_MIN << ") に移行します。" << std::endl;
                    thruster_set_all_pwm(PWM_MIN);
                    latest_gamepad_data = GamepadData{}; // 古いコマンドをクリア
                    currently_in_failsafe = true;
                    // フェイルセーフ起動（接続タイムアウト後）のためプログラムを終了
                    std::cout << "フェイルセーフ起動のためプログラムを終了します。" << std::endl;
                    running = false;
                }
            }
            // recv_len < 0 かつ EAGAIN/EWOULDBLOCK 以外の場合は受信エラー
            if (recv_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                std::cerr << "致命的な受信エラー。ループを継続します..." << std::endl;
            }
        }

        // 3. 制御ロジック (フェイルセーフ中でない場合のみ実行)
        if (!currently_in_failsafe && running) // プログラムが実行中の場合のみ制御ロジックを実行
        {
            current_gyro_data = read_gyro();
            thruster_update(latest_gamepad_data, current_gyro_data);

            // センサーデータ処理 (読み取り、フォーマット、送信) - 一定間隔で実行
            if (loop_counter >= SENSOR_SEND_INTERVAL)
            {
                std::cout << "[SENSOR LOG] " << sensor_buffer << std::endl; // ログは送信時のみ表示

                loop_counter = 0; // カウンターリセット
                if (read_and_format_sensor_data(sensor_buffer, sizeof(sensor_buffer)))
                {
                    std::cout << "[SENSOR LOG] " << sensor_buffer << std::endl;
                    network_send(&net_ctx, sensor_buffer, strlen(sensor_buffer)); // フォーマットされたセンサーデータを送信
                }
                else
                {
                    std::cerr << "センサーデータの読み取り/フォーマットに失敗。" << std::endl;
                }
            }
            else
            {
                loop_counter++; // カウンターインクリメント
            }
        }
        else
        {
            // フェイルセーフ中または最初の接続待機中。スラスターは既に設定済み。
            loop_counter = 0; // 再接続時にセンサーデータが即座に送信されるのを防ぐ
        }

        // // 4. 終了条件チェック (データ受信時のみ Start ボタンを評価)
        // if (just_received_packet && (latest_gamepad_data.buttons & GamepadButton::Start))
        // {
        //     std::cout << "Startボタン検出。終了します。" << std::endl;
        //     running = false;
        // }

        // 5. ループ待機 (CPU負荷軽減とループ頻度調整)
        usleep(10000); // 10000マイクロ秒 = 10ミリ秒待機 (約100Hzのループ周波数)
    }

    // --- クリーンアップ ---
    std::cout << "クリーンアップ処理を開始します..." << std::endl;
    thruster_disable();      // スラスターへのPWM出力を停止
    network_close(&net_ctx); // ネットワークソケットをクローズ
    stop_gstreamer_pipelines(); // GStreamerパイプラインを停止
    std::cout << "プログラム終了。" << std::endl;
    return 0;
}
