#ifndef CONFIG_SYNC_H
#define CONFIG_SYNC_H

#include <mutex>

// グローバル設定オブジェクトを保護するためのミューテックス
extern std::mutex g_config_mutex;

// 設定同期TCPサーバーを別スレッドで起動する
void start_config_sync_server();

// 設定同期TCPサーバーを停止する
void stop_config_sync_server();

#endif // CONFIG_SYNC_H

