#include "gstPipeline.h"
#include <iostream>
#include <string>   // For std::string and std::to_string
#include <thread>   // For std::thread

// --- グローバル変数 ---
// GStreamerパイプラインのインスタンス (カメラ1用)
static GstElement *pipeline1 = nullptr; 
// GStreamerパイプラインのインスタンス (カメラ2用)
static GstElement *pipeline2 = nullptr; 
// GStreamerのメインループ (カメラ1用)。イベント処理やメッセージ処理を行う。
static GMainLoop *main_loop1 = nullptr; 
// GStreamerのメインループ (カメラ2用)
static GMainLoop *main_loop2 = nullptr; 
// main_loop1を実行するためのスレッド
static std::thread loop_thread1; 
// main_loop2を実行するためのスレッド
static std::thread loop_thread2; 

// パイプライン設定を保持するための構造体
struct PipelineConfig {
    std::string device;                 // カメラデバイスのパス (例: "/dev/video0")
    int port;                           // UDP送信先のポート番号
    std::string host = "192.168.6.10";  // UDP送信先のホストIPアドレス (デフォルト値)
    int width = 1280;                   // キャプチャする映像の幅 (デフォルト値)
    int height = 720;                   // キャプチャする映像の高さ (デフォルト値)
    int framerate_num = 30;             // フレームレートの分子 (デフォルト値)
    int framerate_den = 1;              // フレームレートの分母 (デフォルト値)
    bool is_h264_native_source = false; // ソースカメラがH.264ネイティブ出力かどうかのフラグ (trueならエンコード不要)
    int rtp_payload_type = 96;          // RTPペイロードタイプ (デフォルト値)
    int rtp_config_interval = 1;        // rtph264payのconfig-interval および h264parseのconfig-interval (デフォルト値)

    // JPEGソースなど、H.264へのエンコードが必要な場合のx264encパラメータ
    int x264_bitrate = 5000;                     // エンコードビットレート (kbps, デフォルト値)
    std::string x264_tune = "zerolatency";       // x264encのチューニングオプション (デフォルト値)
    std::string x264_speed_preset = "superfast"; // x264encの速度プリセット (デフォルト値)
};

// GMainLoopを指定されたスレッドで実行するための関数
static void run_main_loop(GMainLoop* loop) {
    g_main_loop_run(loop);
}

// 指定された設定に基づいてGStreamerパイプラインを作成し、メインループを開始する関数
static bool create_pipeline(const PipelineConfig& config, GstElement** pipeline_ptr, GMainLoop** loop_ptr) {
    std::string pipeline_str = "v4l2src device=" + config.device + " ! ";

    if (config.is_h264_native_source) {
        // カメラがH.264ネイティブ出力の場合のパイプライン文字列を構築
        // v4l2src -> video/x-h264 caps -> h264parse
        pipeline_str += "video/x-h264,width=" + std::to_string(config.width) +
                        ",height=" + std::to_string(config.height) +
                        ",framerate=" + std::to_string(config.framerate_num) + "/" + std::to_string(config.framerate_den) + " ! "
                        "h264parse config-interval=" + std::to_string(config.rtp_config_interval);
    } else {
        // カメラがJPEG出力など、H.264へのエンコードが必要な場合のパイプライン文字列を構築
        // v4l2src -> image/jpeg caps -> jpegdec -> videoconvert -> x264enc
        pipeline_str += "image/jpeg,width=" + std::to_string(config.width) +
                        ",height=" + std::to_string(config.height) +
                        ",framerate=" + std::to_string(config.framerate_num) + "/" + std::to_string(config.framerate_den) + " ! "
                        "jpegdec ! videoconvert ! "
                        "x264enc tune=" + config.x264_tune +
                        " bitrate=" + std::to_string(config.x264_bitrate) +
                        " speed-preset=" + config.x264_speed_preset;
    }

    // 共通のパイプライン末尾部分 (RTPパッキングとUDP送信) を追加
    // ... ! rtph264pay ! udpsink
    pipeline_str += " ! rtph264pay config-interval=" + std::to_string(config.rtp_config_interval) +
                    " pt=" + std::to_string(config.rtp_payload_type) + " ! "
                    "udpsink host=" + config.host + " port=" + std::to_string(config.port);

    GError* error = nullptr;
    // 構築したパイプライン文字列からGStreamerパイプラインをパース(作成)
    *pipeline_ptr = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!*pipeline_ptr) {
        // パイプライン作成失敗時のエラー処理
        std::cerr << "GStreamerパイプライン作成失敗 (" << config.device << "): " << error->message << std::endl;
        g_error_free(error);
        return false;
    }
    // 作成されたパイプライン文字列をデバッグ出力
    std::cout << "GStreamer pipeline for " << config.device << " (" << config.port << "): " << pipeline_str << std::endl;

    // パイプライン用のGMainLoopを作成
    *loop_ptr = g_main_loop_new(nullptr, FALSE);
    // パイプラインをPLAYING状態に遷移させる
    gst_element_set_state(*pipeline_ptr, GST_STATE_PLAYING);

    return true;
}

// GStreamerパイプラインを開始するメイン関数
bool start_gstreamer_pipelines() {
    // GStreamerライブラリの初期化 (アプリケーション開始時に一度だけ呼び出す)
    gst_init(nullptr, nullptr);

    // カメラ1 (/dev/video2) の設定: H.264ネイティブソースとして設定
    PipelineConfig config1;
    config1.device = "/dev/video2";
    config1.port = 5000;
    config1.is_h264_native_source = true; // H.264ネイティブソースであることを指定
    // その他のパラメータ (解像度、フレームレートなど) はPipelineConfig構造体のデフォルト値を使用

    // カメラ1のパイプラインを作成・起動
    if (!create_pipeline(config1, &pipeline1, &main_loop1)) return false;

    // カメラ2 (/dev/video4) の設定: JPEGソースとして設定 (H.264へのエンコードが必要)
    PipelineConfig config2;
    config2.device = "/dev/video4";
    config2.port = 5001;
    config2.is_h264_native_source = false; // H.264ネイティブではない (エンコードが必要) ことを指定
    // x264encのパラメータはPipelineConfig構造体のデフォルト値を使用

    // カメラ2のパイプラインを作成・起動
    if (!create_pipeline(config2, &pipeline2, &main_loop2)) return false;

    // 各パイプラインのGMainLoopを別々のスレッドで実行開始
    loop_thread1 = std::thread(run_main_loop, main_loop1);
    loop_thread2 = std::thread(run_main_loop, main_loop2);

    std::cout << "GStreamerパイプラインを非同期で起動しました。" << std::endl;
    return true;
}
// GStreamerパイプラインを停止し、リソースを解放する関数
void stop_gstreamer_pipelines() {
    std::cout << "GStreamerパイプラインを停止します..." << std::endl;

    if (pipeline1) {
        // パイプライン1をNULL状態に遷移させて停止
        gst_element_set_state(pipeline1, GST_STATE_NULL);
        // パイプライン1オブジェクトの参照カウントを減らす (不要になれば解放される)
        gst_object_unref(pipeline1);
        pipeline1 = nullptr;
    }
    if (main_loop1) {
        // メインループ1に終了を要求
        g_main_loop_quit(main_loop1);
        // メインループ1を実行しているスレッドが終了するのを待つ
        if (loop_thread1.joinable()) loop_thread1.join();
        // メインループ1オブジェクトの参照カウントを減らす
        g_main_loop_unref(main_loop1);
        main_loop1 = nullptr;
    }

    if (pipeline2) {
        // パイプライン2をNULL状態に遷移させて停止
        gst_element_set_state(pipeline2, GST_STATE_NULL);
        // パイプライン2オブジェクトの参照カウントを減らす
        gst_object_unref(pipeline2);
        pipeline2 = nullptr;
    }
    if (main_loop2) {
        // メインループ2に終了を要求
        g_main_loop_quit(main_loop2);
        // メインループ2を実行しているスレッドが終了するのを待つ
        if (loop_thread2.joinable()) loop_thread2.join();
        // メインループ2オブジェクトの参照カウントを減らす
        g_main_loop_unref(main_loop2);
        main_loop2 = nullptr;
    }

    std::cout << "GStreamerパイプラインを停止しました。" << std::endl;
}
