#ifndef CONFIG_SYNCHRONIZER_H
#define CONFIG_SYNCHRONIZER_H

#include <string>
#include <thread>
#include <atomic>

// 設定が更新され、リロードが必要であることを通知するためのフラグ。
extern std::atomic<bool> g_config_updated_flag;

// ConfigSynchronizerクラスは、設定ファイルの同期とWPFアプリケーションとの通信を管理します。
class ConfigSynchronizer {
public:
    // コンストラクタ: 設定ファイルのパスを受け取ります。
    ConfigSynchronizer(const std::string& config_path);
    // デストラクタ: スレッドを停止し、リソースを解放します。
    ~ConfigSynchronizer();

    // 同期処理を開始します。新しいスレッドを起動します。
    void start();
    // 同期処理を停止します。スレッドのシャットダウンを通知します。
    void stop();

private:
    // 同期処理のメインループ。別スレッドで実行されます。
    void run();
    // 設定ファイルを読み込み、内部マップに格納します。
    bool load_config();
    // 内部マップの現在の設定をファイルに保存します。
    void save_config();
    // 現在の設定データを文字列形式にシリアライズします。
    std::string serialize_config();
    // 受信した文字列データから設定を更新します。
    void update_config_from_string(const std::string& data);
    // 現在の設定をWPFアプリケーションに送信します。
    bool send_config_to_wpf();
    // クライアントからの接続を処理し、設定更新データを受信します。
    void handle_client_connection(int client_sock);
    // WPFアプリケーションからの設定更新を受信するためにリッスンします。
    void receive_config_updates();

    // 設定ファイルのパス。
    std::string m_config_path;
    // 同期処理を実行するスレッド。
    std::thread m_thread;
    // スレッドのシャットダウンを制御するアトミックフラグ。
    std::atomic<bool> m_shutdown_flag;
};

#endif // CONFIG_SYNCHRONIZER_H
