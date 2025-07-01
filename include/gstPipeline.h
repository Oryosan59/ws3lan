#ifndef GST_PIPELINE_H
#define GST_PIPELINE_H

#include <gst/gst.h>
#include <thread>
#include "config.h" // AppConfig を使用するため

// GStreamerパイプラインを開始する
bool start_gstreamer_pipelines(const AppConfig& app_config);

// GStreamerパイプラインを停止する
void stop_gstreamer_pipelines();

#endif // GST_PIPELINE_H