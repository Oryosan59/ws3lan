#ifndef CONFIG_SYNCHRONIZER_H
#define CONFIG_SYNCHRONIZER_H

#include <string>

// ConfigSynchronizerを初期化し、バックグラウンドで起動する
void start_config_synchronizer(const std::string& config_path);

// ConfigSynchronizerを停止する
void stop_config_synchronizer();

#endif // CONFIG_SYNCHRONIZER_H
