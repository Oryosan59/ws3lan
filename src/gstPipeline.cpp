#include "gstPipeline.h"
#include <iostream>
#include <vector>
#include <string>
#include <thread>

// --- グローバル変数 ---
struct PipelineInfo {
    GstElement* pipeline = nullptr;
    GMainLoop* main_loop = nullptr;
    std::thread loop_thread;
};

static std::vector<PipelineInfo> g_pipelines;

// GMainLoopを指定されたスレッドで実行するための関数
static void run_main_loop(GMainLoop* loop) {
    if (loop) {
        g_main_loop_run(loop);
    }
}

// GStreamerパイプラインを開始するメイン関数
bool start_gstreamer_pipelines(const AppConfig& app_config) {
    // GStreamerライブラリの初期化
    gst_init(nullptr, nullptr);

    // 既存のパイプラインがあれば停止・クリア
    stop_gstreamer_pipelines();
    g_pipelines.clear();

    // g_config内のgstreamer_configsをループしてパイプラインを作成
    for (const auto& pair : app_config.gstreamer_configs) {
        const std::string& section_name = pair.first;
        const GStreamerConfig& gst_conf = pair.second;

        std::string pipeline_str = "v4l2src device=" + gst_conf.device + " ! ";

        if (gst_conf.is_h264_native_source) {
            pipeline_str += "video/x-h264,width=" + std::to_string(gst_conf.width) +
                            ",height=" + std::to_string(gst_conf.height) +
                            ",framerate=" + std::to_string(gst_conf.framerate_num) + "/" + std::to_string(gst_conf.framerate_den) + " ! "
                            "h264parse config-interval=" + std::to_string(gst_conf.rtp_config_interval);
        } else {
            pipeline_str += "image/jpeg,width=" + std::to_string(gst_conf.width) +
                            ",height=" + std::to_string(gst_conf.height) +
                            ",framerate=" + std::to_string(gst_conf.framerate_num) + "/" + std::to_string(gst_conf.framerate_den) + " ! "
                            "jpegdec ! videoconvert ! "
                            "x264enc tune=" + gst_conf.x264_tune +
                            " bitrate=" + std::to_string(gst_conf.x264_bitrate) +
                            " speed-preset=" + gst_conf.x264_speed_preset;
        }

        pipeline_str += " ! rtph264pay config-interval=" + std::to_string(gst_conf.rtp_config_interval) +
                        " pt=" + std::to_string(gst_conf.rtp_payload_type) + " ! "
                        "udpsink host=" + app_config.client_host + " port=" + std::to_string(gst_conf.port);

        GError* error = nullptr;
        GstElement* pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

        if (!pipeline) {
            std::cerr << "GStreamerパイプライン作成失敗 (" << section_name << "): " << error->message << std::endl;
            g_error_free(error);
            // 一つのパイプラインが失敗しても、他のパイプラインは続行する
            continue;
        }

        std::cout << "GStreamer pipeline for " << section_name << " (" << gst_conf.device << "): " << pipeline_str << std::endl;

        GMainLoop* main_loop = g_main_loop_new(nullptr, FALSE);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        PipelineInfo info;
        info.pipeline = pipeline;
        info.main_loop = main_loop;
        info.loop_thread = std::thread(run_main_loop, main_loop);
        g_pipelines.push_back(std::move(info));
    }

    std::cout << "GStreamerパイプラインを " << g_pipelines.size() << " 個、非同期で起動しました。" << std::endl;
    return true;
}

// GStreamerパイプラインを停止し、リソースを解放する関数
void stop_gstreamer_pipelines() {
    std::cout << "GStreamerパイプラインを停止します..." << std::endl;

    for (auto& info : g_pipelines) {
        if (info.pipeline) {
            gst_element_set_state(info.pipeline, GST_STATE_NULL);
            gst_object_unref(info.pipeline);
            info.pipeline = nullptr;
        }
        if (info.main_loop) {
            if (g_main_loop_is_running(info.main_loop)) {
                g_main_loop_quit(info.main_loop);
            }
            if (info.loop_thread.joinable()) {
                info.loop_thread.join();
            }
            g_main_loop_unref(info.main_loop);
            info.main_loop = nullptr;
        }
    }
    g_pipelines.clear();

    std::cout << "GStreamerパイプラインを停止しました。" << std::endl;
}