// ConfigSynchronizer.cpp
// 設定同期クラスの実装ファイル
#include "config_synchronizer.h"
#include "config.h" // g_configとloadConfig用
#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <vector>
#include <sstream>
#include <fstream>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <signal.h>

// 文字列の両端から空白文字をトリムするヘルパー関数
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(first, (last - first + 1));
}

// 同期処理用の設定データを保持するグローバルマップ
static std::map<std::string, std::map<std::string, std::string>> g_sync_config_data;
// 設定データへのアクセスを保護するためのミューテックス
static std::mutex g_config_mutex;

ConfigSynchronizer::ConfigSynchronizer(const std::string& config_path)
    : m_config_path(config_path), m_shutdown_flag(false) {}

// デストラクタ: stop()を呼び出してスレッドを安全に終了させます。
ConfigSynchronizer::~ConfigSynchronizer() {
    stop();
}

// 同期処理を開始します。新しいスレッドを作成し、runメソッドを実行します。
void ConfigSynchronizer::start() {
    m_thread = std::thread(&ConfigSynchronizer::run, this);
}

// 同期処理を停止します。シャットダウンフラグをtrueに設定し、スレッドが終了するのを待ちます。
void ConfigSynchronizer::stop() {
    m_shutdown_flag.store(true);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

// 同期処理のメインループ。設定のロード、WPFへの初期設定送信、および更新の受信を行います。
void ConfigSynchronizer::run() {
    std::cout << "ConfigSynchronizer thread started." << std::endl;
    // 設定のロードを試みます。失敗した場合はエラーを出力し、終了します。
    if (!load_config()) {
        std::cerr << "Failed to load config for synchronizer." << std::endl;
        return;
    }

    // 成功するまでWPFへの初期設定の接続と送信を試行し続けます。
    while (!m_shutdown_flag.load()) {
        std::cout << "Attempting to connect to WPF to send initial configuration..." << std::endl;
        if (send_config_to_wpf()) {
            std::cout << "Initial configuration sent successfully." << std::endl;
            break; // 成功したらループを抜けます
        }
        std::cerr << "Failed to send initial configuration. Retrying in 5 seconds..." << std::endl;
        
        // 5秒待機してから再試行しますが、シャットダウンフラグを定期的にチェックします。
        for (int i = 0; i < 5; ++i) {
            if (m_shutdown_flag.load()) break;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // シャットダウン中でない場合、更新の受信に進みます。
    if (!m_shutdown_flag.load()) {
        receive_config_updates();
    }

    std::cout << "ConfigSynchronizer thread finished." << std::endl;
}

// 設定ファイルを読み込み、g_sync_config_dataに格納します。
bool ConfigSynchronizer::load_config() {
    std::ifstream file(m_config_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open config file 
'" << m_config_path << "'" << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> lock(g_config_mutex); // ミューテックスで保護
    g_sync_config_data.clear(); // 既存のデータをクリア
    std::string current_section; // 現在のセクション名
    std::string line;

    // ファイルを行ごとに読み込みます
    while (std::getline(file, line)) {
        line = trim(line); // 行をトリム
        if (line.empty() || line[0] == '#' || line[0] == ';') { // 空行、コメント行はスキップ
            continue;
        }
        if (line[0] == '[' && line.back() == ']') { // セクションヘッダーの検出
            current_section = line.substr(1, line.length() - 2);
        } else { // キーと値のペアの検出
            size_t eq_pos = line.find('=');
            if (eq_pos != std::string::npos) {
                std::string key = trim(line.substr(0, eq_pos));
                std::string value = trim(line.substr(eq_pos + 1));
                g_sync_config_data[current_section][key] = value;
            }
        }
    }
    return true;
}

// 現在のg_sync_config_dataの内容を設定ファイルに保存します。
void ConfigSynchronizer::save_config() {
    std::lock_guard<std::mutex> lock(g_config_mutex); // ミューテックスで保護
    std::ofstream file(m_config_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open config file for writing." << std::endl;
        return;
    }

    // 各セクションとキー-値のペアをファイルに書き込みます
    for (const auto& section_pair : g_sync_config_data) {
        file << "[" << section_pair.first << "]" << std::endl;
        for (const auto& key_value_pair : section_pair.second) {
            file << key_value_pair.first << " = " << key_value_pair.second << std::endl;
        }
        file << std::endl; // セクション間に空行を挿入
    }
    std::cout << "Configuration saved to " << m_config_path << std::endl;
}

// g_sync_config_dataの内容を文字列にシリアライズします。WPFへの送信形式に合わせます。
std::string ConfigSynchronizer::serialize_config() {
    std::lock_guard<std::mutex> lock(g_config_mutex); // ミューテックスで保護
    std::stringstream ss;
    // 各セクションとキー-値のペアを文字列ストリームに書き込みます
    for (const auto& section_pair : g_sync_config_data) {
        for (const auto& key_value_pair : section_pair.second) {
            ss << "[" << section_pair.first << "]"
               << key_value_pair.first << "=" << key_value_pair.second << "\n";
        }
    }
    std::string content = ss.str();
    // データ長をプレフィックスとして追加して返します
    return std::to_string(content.length()) + "\n" + content;
}

// 受信した文字列データからg_sync_config_dataを更新します。
void ConfigSynchronizer::update_config_from_string(const std::string& data) {
    std::stringstream ss(data);
    std::string line;
    int updates_count = 0;

    {
        std::lock_guard<std::mutex> lock(g_config_mutex); // ミューテックスで保護
        // 行ごとにデータを解析し、設定を更新します
        while (std::getline(ss, line, '\n')) {
            if (line.empty() || line[0] != '[') continue;

            size_t section_end = line.find(']');
            size_t equals_pos = line.find('=', section_end);

            if (section_end != std::string::npos && equals_pos != std::string::npos) {
                std::string section = line.substr(1, section_end - 1);
                std::string key = line.substr(section_end + 1, equals_pos - (section_end + 1));
                std::string value = line.substr(equals_pos + 1);
                g_sync_config_data[section][key] = value;
                updates_count++;
            }
        }
    }

    // 更新があった場合、設定を保存し、メインスレッドにリロードを通知します。
    if (updates_count > 0) {
        std::cout << "Updated " << updates_count << " config items from WPF." << std::endl;
        save_config();
        // メインスレッドに設定のリロードを通知するフラグを設定
        g_config_updated_flag.store(true);
    }
}

// WPFアプリケーションに現在の設定を送信します。
bool ConfigSynchronizer::send_config_to_wpf() {
    std::string host;
    int port = 0;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex); // ミューテックスで保護
        // 設定からWPFのホストとポートを取得します
        if (g_sync_config_data.count("CONFIG_SYNC") && g_sync_config_data["CONFIG_SYNC"].count("WPF_HOST")) {
            host = g_sync_config_data["CONFIG_SYNC"]["WPF_HOST"];
        } else {
            std::cerr << "WPF_HOST not found in config." << std::endl;
            return false;
        }
        if (g_sync_config_data.count("CONFIG_SYNC") && g_sync_config_data["CONFIG_SYNC"].count("WPF_RECV_PORT")) {
            port = std::stoi(g_sync_config_data["CONFIG_SYNC"]["WPF_RECV_PORT"]);
        } else {
            std::cerr << "WPF_RECV_PORT not found in config." << std::endl;
            return false;
        }
    }


    // ソケットを作成します
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return false;
    }

    // サーバーアドレス構造体を設定します
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

    // サーバーに接続します
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection to WPF failed" << std::endl;
        close(sock);
        return false;
    }

    // 設定データをシリアライズして送信します
    std::string config_str = serialize_config();
    send(sock, config_str.c_str(), config_str.length(), 0);
    std::cout << "Sent config to WPF." << std::endl;

    close(sock);
    return true;
}

// クライアント（WPF）からの接続を処理し、設定更新データを受信します。
void ConfigSynchronizer::handle_client_connection(int client_sock) {
    // クライアントソケットに受信タイムアウトを設定します。これにより、クライアントがデータを送信しない場合に
    // スレッドがrecv()呼び出しで無期限にブロックされるのを防ぎます。これはアプリケーションのクリーンな
    // シャットダウンを可能にするために重要です。
    struct timeval tv;
    tv.tv_sec = 1;  // 1秒のタイムアウト
    tv.tv_usec = 0;
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    char buffer[4096] = {0};
    std::string header;
    char c;

    // ヘッダー（データ長）を読み取ります
    while (recv(client_sock, &c, 1, 0) > 0) {
        if (c == '\n') break;
        header += c;
    }

    if (header.empty()) {
        close(client_sock);
        return;
    }

    // 期待されるデータ長を解析します
    size_t expected_length = std::stoull(header);
    std::string received_data;
    received_data.reserve(expected_length);
    size_t total_received = 0;

    // データ本体を受信します
    while (total_received < expected_length) {
        ssize_t bytes_received = recv(client_sock, buffer, sizeof(buffer), 0);
        if (bytes_received <= 0) {
            std::cerr << "Error receiving config data body: connection closed, timed out, or error." << std::endl;
            close(client_sock);
            return;
        }
        received_data.append(buffer, bytes_received);
        total_received += bytes_received;
    }

    // シャットダウン中でない場合、受信したデータで設定を更新します
    if (!m_shutdown_flag.load()) {
        std::cout << "Received config data from WPF." << std::endl;
        update_config_from_string(received_data);
    }

    close(client_sock);
}

// WPFアプリケーションからの設定更新を受信するためにリッスンします。
void ConfigSynchronizer::receive_config_updates() {
    int port = 0;
    {
        std::lock_guard<std::mutex> lock(g_config_mutex); // ミューテックスで保護
        // 設定からC++側の受信ポートを取得します
        if (g_sync_config_data.count("CONFIG_SYNC") && g_sync_config_data["CONFIG_SYNC"].count("CPP_RECV_PORT")) {
            port = std::stoi(g_sync_config_data["CONFIG_SYNC"]["CPP_RECV_PORT"]);
        } else {
            std::cerr << "CPP_RECV_PORT not found in config." << std::endl;
            return;
        }
    }

    // リッスンソケットを作成します
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        std::cerr << "Error creating listening socket." << std::endl;
        return;
    }

    // ソケットオプションを設定します（アドレスの再利用を許可）
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // サーバーアドレス構造体を設定します
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // すべてのインターフェースからの接続を許可
    server_addr.sin_port = htons(port);

    // ソケットをポートにバインドします
    if (bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Bind failed on port " << port << std::endl;
        close(listen_sock);
        return;
    }

    // 接続をリッスンします
    if (listen(listen_sock, 5) < 0) {
        std::cerr << "Listen failed." << std::endl;
        close(listen_sock);
        return;
    }

    std::cout << "Listening for config updates on port " << port << std::endl;

    // シャットダウンフラグが設定されるまで接続を待ち続けます
    while (!m_shutdown_flag.load()) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listen_sock, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 1; // 1秒のタイムアウト
        timeout.tv_usec = 0;

        // select()を使用して、ソケットのアクティビティを監視します
        int activity = select(listen_sock + 1, &readfds, nullptr, nullptr, &timeout);

        if (activity < 0 && errno != EINTR) { // エラーが発生した場合（EINTR以外）はループを抜けます
            break;
        }

        if (activity > 0 && FD_ISSET(listen_sock, &readfds)) { // 接続要求があった場合
            int client_sock = accept(listen_sock, nullptr, nullptr); // 接続を受け入れます
            if (client_sock >= 0) {
                handle_client_connection(client_sock); // クライアント接続を処理します
            }
        }
    }

    close(listen_sock);
}
